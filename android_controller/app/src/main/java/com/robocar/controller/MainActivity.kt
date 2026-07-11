package com.robocar.controller

import android.app.Activity
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.view.MotionEvent
import android.widget.Button
import android.widget.EditText
import android.widget.TextView
import java.net.Socket
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicInteger

// Speaks the same "V <vx> <vy> <w>\n" protocol used by the Pi<->Pico USB
// serial link (see ARCHITECTURE.md): vx forward(+)/back(-), vy strafe
// right(+)/left(-), w yaw CCW(+)/CW(-), each -100..100.
private const val SPEED = 50
private const val STREAM_INTERVAL_MS = 100L // 10 Hz, feeds the Pico's watchdog

class MainActivity : Activity() {

    private val connected = AtomicBoolean(false)
    private val vx = AtomicInteger(0)
    private val vy = AtomicInteger(0)
    private val w = AtomicInteger(0)
    private val mainHandler = Handler(Looper.getMainLooper())

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        val ipField = findViewById<EditText>(R.id.ipField)
        val portField = findViewById<EditText>(R.id.portField)
        val statusText = findViewById<TextView>(R.id.statusText)

        findViewById<Button>(R.id.connectButton).setOnClickListener {
            val ip = ipField.text.toString().trim()
            val port = portField.text.toString().trim().toIntOrNull()
            if (port == null) {
                statusText.text = "Bad port"
            } else {
                connect(ip, port, statusText)
            }
        }

        setupDirectionButton(R.id.btnForward) { vx.set(SPEED); vy.set(0); w.set(0) }
        setupDirectionButton(R.id.btnBackward) { vx.set(-SPEED); vy.set(0); w.set(0) }
        setupDirectionButton(R.id.btnStrafeLeft) { vx.set(0); vy.set(-SPEED); w.set(0) }
        setupDirectionButton(R.id.btnStrafeRight) { vx.set(0); vy.set(SPEED); w.set(0) }
        setupDirectionButton(R.id.btnTurnLeft) { vx.set(0); vy.set(0); w.set(SPEED) }
        setupDirectionButton(R.id.btnTurnRight) { vx.set(0); vy.set(0); w.set(-SPEED) }

        findViewById<Button>(R.id.btnStop).setOnClickListener {
            vx.set(0); vy.set(0); w.set(0)
        }
    }

    private fun setupDirectionButton(id: Int, onPress: () -> Unit) {
        findViewById<Button>(id).setOnTouchListener { _, event ->
            when (event.action) {
                MotionEvent.ACTION_DOWN -> onPress()
                MotionEvent.ACTION_UP, MotionEvent.ACTION_CANCEL -> {
                    vx.set(0); vy.set(0); w.set(0)
                }
            }
            true
        }
    }

    private fun connect(ip: String, port: Int, statusText: TextView) {
        connected.set(false)
        Thread {
            try {
                val socket = Socket(ip, port)
                val out = socket.getOutputStream()
                connected.set(true)
                mainHandler.post { statusText.text = "Connected to $ip:$port" }

                while (connected.get()) {
                    val line = "V ${vx.get()} ${vy.get()} ${w.get()}\n"
                    out.write(line.toByteArray())
                    out.flush()
                    Thread.sleep(STREAM_INTERVAL_MS)
                }
                socket.close()
            } catch (e: Exception) {
                connected.set(false)
                mainHandler.post { statusText.text = "Connect failed: ${e.message}" }
            }
        }.start()
    }

    override fun onDestroy() {
        super.onDestroy()
        connected.set(false)
    }
}
