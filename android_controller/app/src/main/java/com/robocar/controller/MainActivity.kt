package com.robocar.controller

import android.app.Activity
import android.graphics.BitmapFactory
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.view.MotionEvent
import android.widget.Button
import android.widget.EditText
import android.widget.ImageView
import android.widget.TextView
import org.json.JSONObject
import java.io.BufferedInputStream
import java.io.ByteArrayOutputStream
import java.net.HttpURLConnection
import java.net.URL
import java.util.concurrent.Executors
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicInteger

// Talks HTTP to cone_visitor.py on the Pi (which hosts the WiFi AP):
//   GET /stream            annotated MJPEG video
//   GET /start  /stop      autonomy on/off
//   GET /drive?vx=&vy=&w=  manual drive, only honored while autonomy is
//                          stopped; expires server-side after 0.5 s so we
//                          re-send while a button is held
//   GET /status            JSON {running, state, cones, distance_cm}
// Velocity convention (ARCHITECTURE.md): vx fwd(+), vy strafe right(+),
// w yaw CCW(+), each -100..100.
private const val SPEED = 50
private const val DRIVE_INTERVAL_MS = 100L   // 10 Hz while a button is held
private const val STATUS_INTERVAL_MS = 500L

class MainActivity : Activity() {

    private val connected = AtomicBoolean(false)
    private val vx = AtomicInteger(0)
    private val vy = AtomicInteger(0)
    private val w = AtomicInteger(0)
    private val mainHandler = Handler(Looper.getMainLooper())
    private val httpExecutor = Executors.newSingleThreadExecutor()
    @Volatile private var baseUrl = ""

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        val ipField = findViewById<EditText>(R.id.ipField)
        val portField = findViewById<EditText>(R.id.portField)
        val statusText = findViewById<TextView>(R.id.statusText)
        val videoView = findViewById<ImageView>(R.id.videoView)

        findViewById<Button>(R.id.connectButton).setOnClickListener {
            val ip = ipField.text.toString().trim()
            val port = portField.text.toString().trim().toIntOrNull()
            if (port == null) {
                statusText.text = "Bad port"
            } else {
                connect(ip, port, statusText, videoView)
            }
        }

        findViewById<Button>(R.id.btnStart).setOnClickListener { get("/start") }
        findViewById<Button>(R.id.btnStopAuto).setOnClickListener { get("/stop") }

        setupDirectionButton(R.id.btnForward) { vx.set(SPEED); vy.set(0); w.set(0) }
        setupDirectionButton(R.id.btnBackward) { vx.set(-SPEED); vy.set(0); w.set(0) }
        setupDirectionButton(R.id.btnStrafeLeft) { vx.set(0); vy.set(-SPEED); w.set(0) }
        setupDirectionButton(R.id.btnStrafeRight) { vx.set(0); vy.set(SPEED); w.set(0) }
        setupDirectionButton(R.id.btnTurnLeft) { vx.set(0); vy.set(0); w.set(SPEED) }
        setupDirectionButton(R.id.btnTurnRight) { vx.set(0); vy.set(0); w.set(-SPEED) }

        findViewById<Button>(R.id.btnStop).setOnClickListener {
            vx.set(0); vy.set(0); w.set(0)
            get("/drive?vx=0&vy=0&w=0")
        }
    }

    private fun setupDirectionButton(id: Int, onPress: () -> Unit) {
        findViewById<Button>(id).setOnTouchListener { _, event ->
            when (event.action) {
                MotionEvent.ACTION_DOWN -> onPress()
                MotionEvent.ACTION_UP, MotionEvent.ACTION_CANCEL -> {
                    vx.set(0); vy.set(0); w.set(0)
                    get("/drive?vx=0&vy=0&w=0")   // stop immediately, don't
                }                                  // wait for the Pi timeout
            }
            true
        }
    }

    /** Fire-and-forget GET on the background executor. */
    private fun get(path: String) {
        val base = baseUrl
        if (base.isEmpty()) return
        httpExecutor.execute {
            try {
                val conn = URL(base + path).openConnection() as HttpURLConnection
                conn.connectTimeout = 2000
                conn.readTimeout = 2000
                conn.inputStream.use { it.readBytes() }
                conn.disconnect()
            } catch (_: Exception) {
            }
        }
    }

    private fun connect(ip: String, port: Int, statusText: TextView, videoView: ImageView) {
        connected.set(false)   // stops previous threads
        baseUrl = "http://$ip:$port"
        connected.set(true)
        statusText.text = "Connecting to $baseUrl ..."
        streamVideo(videoView, statusText)
        pollStatus(statusText)
        streamDrive()
    }

    /** Read the MJPEG multipart stream and show frames in the ImageView. */
    private fun streamVideo(videoView: ImageView, statusText: TextView) {
        val base = baseUrl
        Thread {
            try {
                val conn = URL("$base/stream").openConnection() as HttpURLConnection
                conn.connectTimeout = 3000
                conn.readTimeout = 5000
                val input = BufferedInputStream(conn.inputStream, 64 * 1024)
                while (connected.get() && base == baseUrl) {
                    // part headers end at an empty line; grab Content-Length
                    var length = -1
                    while (true) {
                        val line = readLine(input) ?: throw Exception("stream ended")
                        if (line.startsWith("Content-Length", ignoreCase = true)) {
                            length = line.substringAfter(":").trim().toInt()
                        }
                        if (line.isEmpty() && length > 0) break
                    }
                    val jpeg = ByteArray(length)
                    var off = 0
                    while (off < length) {
                        val n = input.read(jpeg, off, length - off)
                        if (n < 0) throw Exception("stream ended")
                        off += n
                    }
                    val bmp = BitmapFactory.decodeByteArray(jpeg, 0, length)
                    if (bmp != null) {
                        mainHandler.post { videoView.setImageBitmap(bmp) }
                    }
                }
                conn.disconnect()
            } catch (e: Exception) {
                if (connected.get() && base == baseUrl) {
                    mainHandler.post { statusText.text = "Video lost: ${e.message}" }
                }
            }
        }.start()
    }

    /** Read one \r\n-terminated ASCII line from the stream. */
    private fun readLine(input: BufferedInputStream): String? {
        val buf = ByteArrayOutputStream()
        while (true) {
            val b = input.read()
            if (b < 0) return null
            if (b == '\n'.code) break
            if (b != '\r'.code) buf.write(b)
        }
        return buf.toString("US-ASCII")
    }

    /** Poll /status and show it. */
    private fun pollStatus(statusText: TextView) {
        val base = baseUrl
        Thread {
            while (connected.get() && base == baseUrl) {
                try {
                    val conn = URL("$base/status").openConnection() as HttpURLConnection
                    conn.connectTimeout = 2000
                    conn.readTimeout = 2000
                    val body = conn.inputStream.use { it.readBytes().toString(Charsets.UTF_8) }
                    conn.disconnect()
                    val json = JSONObject(body)
                    val running = json.getBoolean("running")
                    val state = json.getString("state")
                    val cones = json.getInt("cones")
                    val dist = if (json.isNull("distance_cm")) "--"
                               else "${json.getInt("distance_cm")} cm"
                    mainHandler.post {
                        statusText.text =
                            (if (running) "RUNNING" else "stopped") +
                            "  |  $state  |  cones: $cones  |  dist: $dist"
                    }
                } catch (e: Exception) {
                    mainHandler.post { statusText.text = "No status: ${e.message}" }
                }
                Thread.sleep(STATUS_INTERVAL_MS)
            }
        }.start()
    }

    /** While a drive button is held, re-send the command at 10 Hz
     *  (the Pi expires manual commands after 0.5 s as a dead-man switch). */
    private fun streamDrive() {
        val base = baseUrl
        Thread {
            while (connected.get() && base == baseUrl) {
                val x = vx.get(); val y = vy.get(); val z = w.get()
                if (x != 0 || y != 0 || z != 0) {
                    get("/drive?vx=$x&vy=$y&w=$z")
                }
                Thread.sleep(DRIVE_INTERVAL_MS)
            }
        }.start()
    }

    override fun onDestroy() {
        super.onDestroy()
        connected.set(false)
        httpExecutor.shutdown()
    }
}
