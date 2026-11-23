const BUTTON_IDS = {
    "0": "TcHmiButton_33", // CLOSE
    "1": "TcHmiButton_34", // OPEN
};

// Mouse press duration.
const PRESS_MS = 100;

// Minimum delay before allowing OPEN->CLOSE or CLOSE->OPEN.
// Browser/HMI latency is about 0.5 s, so use 0.8 s.
const MIN_STATE_CHANGE_DELAY_MS = 800;

// Small pause after every click.
const POST_CLICK_SETTLE_MS = 100;

// WebSocket reconnect delay.
const RECONNECT_MS = 1000;

let lastApplied = null;
let lastAppliedAt = 0;

let pending = null;
let busy = false;

function sleep(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
}

function normalizeCommand(x) {
    x = String(x).trim().toLowerCase();

    if (x === "1" || x === "open" || x === "true") {
        return "1";
    }

    if (x === "0" || x === "close" || x === "closed" || x === "false") {
        return "0";
    }

    console.warn("[KOALA JS] Unknown gripper command:", x);
    return null;
}

function commandName(cmd) {
    return cmd === "1" ? "OPEN" : "CLOSE";
}

function makeMouseEvent(type, el) {
    const rect = el.getBoundingClientRect();

    return new MouseEvent(type, {
        bubbles: true,
        cancelable: true,
        view: window,
        button: 0,
        buttons: type === "mousedown" ? 1 : 0,
        clientX: rect.left + rect.width / 2,
        clientY: rect.top + rect.height / 2,
    });
}

async function pressButton(cmd) {
    const id = BUTTON_IDS[cmd];
    const btn = document.getElementById(id);

    if (!btn) {
        console.warn("[KOALA JS] Button not found:", id);
        return false;
    }

    console.log(`[KOALA JS] Pressing ${commandName(cmd)} button: ${id}`);

    btn.dispatchEvent(makeMouseEvent("mousedown", btn));
    await sleep(PRESS_MS);
    btn.dispatchEvent(makeMouseEvent("mouseup", btn));

    // Some HMI controls listen to click explicitly.
    btn.dispatchEvent(makeMouseEvent("click", btn));

    return true;
}

async function waitIfChangingState(next) {
    if (lastApplied === null) {
        return;
    }

    if (next === lastApplied) {
        return;
    }

    const elapsed = Date.now() - lastAppliedAt;
    const remaining = MIN_STATE_CHANGE_DELAY_MS - elapsed;

    if (remaining > 0) {
        console.log(
            `[KOALA JS] Waiting ${remaining} ms before opposite command. ` +
            `last=${commandName(lastApplied)}, next=${commandName(next)}`
        );
        await sleep(remaining);
    }
}

async function applyCommand(cmd) {
    if (cmd === null) {
        return;
    }

    // Latest command wins while busy/waiting.
    pending = cmd;

    if (busy) {
        console.log(
            `[KOALA JS] Busy; stored pending command: ${commandName(pending)}`
        );
        return;
    }

    busy = true;

    try {
        while (pending !== null) {
            let next = pending;
            pending = null;

            // Ignore duplicate OPEN/OPEN/OPEN or CLOSE/CLOSE/CLOSE.
            if (next === lastApplied) {
                console.log(
                    `[KOALA JS] Ignoring duplicate command: ${commandName(next)}`
                );
                continue;
            }

            await waitIfChangingState(next);

            // During the forced wait, a newer command may have arrived.
            // Use the newest command instead of the stale one.
            if (pending !== null) {
                next = pending;
                pending = null;

                if (next === lastApplied) {
                    console.log(
                        `[KOALA JS] Pending command equals current state; ignoring: ${commandName(next)}`
                    );
                    continue;
                }

                await waitIfChangingState(next);
            }

            const ok = await pressButton(next);

            if (ok) {
                lastApplied = next;
                lastAppliedAt = Date.now();

                console.log(
                    `[KOALA JS] Applied command: ${commandName(next)}`
                );
            }

            await sleep(POST_CLICK_SETTLE_MS);
        }
    } finally {
        busy = false;

        // A command may have arrived just after the while loop ended.
        if (pending !== null) {
            const nextPending = pending;
            pending = null;
            applyCommand(nextPending);
        }
    }
}

function connect() {
    const socket = new WebSocket("ws://localhost:6789");

    socket.onopen = function() {
        console.log("[KOALA JS] WebSocket connected");
    };

    socket.onmessage = function(event) {
        const cmd = normalizeCommand(event.data);

        console.log(
            "[KOALA JS] Received command:",
            event.data,
            "normalized:",
            cmd
        );

        applyCommand(cmd);
    };

    socket.onclose = function() {
        console.warn("[KOALA JS] WebSocket closed. Reconnecting...");
        setTimeout(connect, RECONNECT_MS);
    };

    socket.onerror = function(err) {
        console.warn("[KOALA JS] WebSocket error:", err);
    };
}

connect();
