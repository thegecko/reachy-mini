/**
 * reachy-mini.js — Browser SDK for controlling a Reachy Mini robot over WebRTC.
 * https://github.com/pollen-robotics/reachy-mini
 *
 * QUICK START
 * ───────────
 *   import { ReachyMini } from "./reachy-mini.js";
 *   const robot = new ReachyMini({ host: "reachy-mini.local" });
 *
 *   // 1. Connect to the robot's local signaling server (WebSocket)
 *   await robot.connect();
 *
 *   // 2. Pick a robot once the list arrives
 *   robot.addEventListener("robotsChanged", (e) => {
 *       const robots = e.detail.robots;  // [{ id, meta: { name } }, ...]
 *   });
 *
 *   // 3. Start a WebRTC session (resolves when video + data channel ready)
 *   const detach = robot.attachVideo(document.querySelector("video"));
 *   await robot.startSession(robotId);
 *
 *   // 4. Send commands
 *   robot.setHeadPose(0, 10, -5);    // roll, pitch, yaw in degrees
 *   robot.setAntennas(30, -30);       // right, left in degrees
 *   robot.playSound("wake_up.wav");   // filename on robot
 *
 *   // 5. Receive live state (emitted every ~500 ms while streaming)
 *   robot.addEventListener("state", (e) => {
 *       const { head, antennas } = e.detail;
 *       // head:     { roll, pitch, yaw }   — degrees
 *       // antennas: { right, left }        — degrees
 *   });
 *
 *   // 6. Audio controls
 *   robot.setAudioMuted(false);   // unmute robot speaker (muted by default)
 *   robot.setMicMuted(false);     // unmute your mic → robot speaker (if supported)
 *
 *   // 7. Cleanup
 *   detach();                      // remove video binding
 *   await robot.stopSession();     // back to 'connected'
 *   robot.disconnect();            // back to 'disconnected'
 *
 *
 * STATE MACHINE
 * ─────────────
 *   'disconnected' ──connect()──▸ 'connected' ──startSession()──▸ 'streaming'
 *        ▴ disconnect()                ▴ stopSession()
 *        └─────────────────────────────┘
 *
 *
 * CONSTRUCTOR OPTIONS
 * ───────────────────
 *   new ReachyMini({
 *     host:             string,   // default: current hostname or "reachy-mini.local"
 *     signalingUrl:     string,   // default: ws://<host>:8443
 *     daemonUrl:        string,   // default: http://<host>:8000
 *     enableMicrophone: boolean,  // default: true — acquire mic for bidirectional audio
 *   })
 *
 *
 * READ-ONLY PROPERTIES
 * ────────────────────
 *   .state            "disconnected" | "connected" | "streaming"
 *   .robots           Array<{ id: string, meta: { name: string } }>
 *   .robotState       { head: { roll, pitch, yaw }, antennas: { right, left } }  (degrees)
 *   .username         string | null     — always "local" in local mode
 *   .isAuthenticated  boolean           — always true in local mode
 *   .micSupported     boolean           — true if robot offers bidirectional audio
 *   .micMuted         boolean           — your microphone mute state
 *   .audioMuted       boolean           — robot speaker mute state (local)
 *
 *
 * EVENTS  (EventTarget — use addEventListener)
 * ──────────────────────────────────────────────
 *   "connected"       { peerId: string }
 *   "disconnected"    { reason: string }
 *   "robotsChanged"   { robots: Array<{ id, meta }> }
 *   "streaming"       { sessionId: string, robotId: string }
 *   "sessionStopped"  { reason: string }
 *   "state"           { head: { roll, pitch, yaw }, antennas: { right, left } }
 *   "videoTrack"      { track: MediaStreamTrack, stream: MediaStream }
 *   "micSupported"    { supported: boolean }
 *   "error"           { source: "signaling"|"webrtc"|"robot", error: Error|string }
 *
 *
 * EXPORTS
 * ───────
 *   export default ReachyMini;
 *   export { ReachyMini, rpyToMatrix, matrixToRpy, degToRad, radToDeg };
 */

// ─── Math utilities ──────────────────────────────────────────────────────────

/** @param {number} deg @returns {number} */
export function degToRad(deg) { return deg * Math.PI / 180; }

/** @param {number} rad @returns {number} */
export function radToDeg(rad) { return rad * 180 / Math.PI; }

/**
 * Roll/pitch/yaw (degrees) → 4×4 rotation matrix (ZYX convention).
 * This is the wire format for the robot's `set_target` command.
 * @param {number} rollDeg  @param {number} pitchDeg  @param {number} yawDeg
 * @returns {number[][]} 4×4 matrix
 */
export function rpyToMatrix(rollDeg, pitchDeg, yawDeg) {
    const r = degToRad(rollDeg), p = degToRad(pitchDeg), y = degToRad(yawDeg);
    const cy = Math.cos(y), sy = Math.sin(y);
    const cp = Math.cos(p), sp = Math.sin(p);
    const cr = Math.cos(r), sr = Math.sin(r);
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, 0],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, 0],
        [-sp,     cp * sr,                cp * cr,                0],
        [0,       0,                      0,                      1],
    ];
}

/**
 * Rotation matrix (3×3 or 4×4) → { roll, pitch, yaw } in degrees.
 * @param {number[][]} m  @returns {{ roll: number, pitch: number, yaw: number }}
 */
export function matrixToRpy(m) {
    return {
        roll:  radToDeg(Math.atan2(m[2][1], m[2][2])),
        pitch: radToDeg(Math.asin(-m[2][0])),
        yaw:   radToDeg(Math.atan2(m[1][0], m[0][0])),
    };
}

// ─── Internal helpers ────────────────────────────────────────────────────────

/** Check if the audio m= section of an SDP has a=sendrecv (bidirectional audio). */
function sdpHasAudioSendRecv(sdp) {
    const lines = sdp.split('\r\n');
    let inAudio = false;
    for (const line of lines) {
        if (line.startsWith('m=audio')) inAudio = true;
        else if (line.startsWith('m=')) inAudio = false;
        if (inAudio && line === 'a=sendrecv') return true;
    }
    return false;
}

function getDefaultHost() {
    return 'reachy-mini.local';
}

function normalizeHost(host) {
    if (!host || typeof host !== 'string') return getDefaultHost();
    const trimmed = host.trim();
    if (!trimmed) return getDefaultHost();
    try {
        if (/^https?:\/\//i.test(trimmed) || /^wss?:\/\//i.test(trimmed)) {
            const parsed = new URL(trimmed);
            return parsed.host;
        }
    } catch (_) {
        // Fall through to raw host parsing.
    }
    return trimmed.replace(/^\/+|\/+$/g, '').replace(/\/$/, '');
}

function buildLocalSignalingUrl(host) {
    const normalized = normalizeHost(host);
    if (/^wss?:\/\//i.test(normalized)) return normalized;
    const secure = typeof window !== 'undefined' && window.location.protocol === 'https:';
    return `${secure ? 'wss' : 'ws'}://${normalized.includes(':') ? normalized : `${normalized}:8443`}`;
}

function buildLocalDaemonUrl(host) {
    const normalized = normalizeHost(host);
    if (/^https?:\/\//i.test(normalized)) return normalized;
    const secure = typeof window !== 'undefined' && window.location.protocol === 'https:';
    return `${secure ? 'https' : 'http'}://${normalized.includes(':') ? normalized : `${normalized}:8000`}`;
}

function normalizeSessionDescription(sdp) {
    if (!sdp) return null;
    if (typeof sdp === 'string') return { type: 'answer', sdp };
    if (typeof sdp === 'object' && typeof sdp.type === 'string' && typeof sdp.sdp === 'string') {
        return sdp;
    }
    return null;
}

function sdpSupportsMicrophone(sdp) {
    if (!sdp || typeof sdp !== 'string') return false;
    const lines = sdp.split('\r\n');
    let inAudio = false;
    let direction = 'sendrecv';
    for (const line of lines) {
        if (line.startsWith('m=audio')) {
            inAudio = true;
            direction = 'sendrecv';
            continue;
        }
        if (line.startsWith('m=')) {
            inAudio = false;
            continue;
        }
        if (!inAudio) continue;
        if (line === 'a=sendrecv' || line === 'a=recvonly' || line === 'a=sendonly' || line === 'a=inactive') {
            direction = line.slice(2);
        }
    }
    return direction === 'sendrecv' || direction === 'recvonly';
}

// ─── ReachyMini class ────────────────────────────────────────────────────────

export class ReachyMini extends EventTarget {

    /** @param {{ host?: string, signalingUrl?: string, daemonUrl?: string, enableMicrophone?: boolean }} [options] */
    constructor(options = {}) {
        super();
        this._host = normalizeHost(options.host);
        this._signalingUrl = options.signalingUrl || buildLocalSignalingUrl(this._host);
        this._daemonUrl = options.daemonUrl || buildLocalDaemonUrl(this._host);
        this._enableMicrophone = options.enableMicrophone !== false;

        this._state = 'disconnected';                 // 'disconnected' | 'connected' | 'streaming'
        this._robots = [];                             // latest robot list from signaling
        this._robotState = {                           // updated every ~500 ms while streaming
            head: { roll: 0, pitch: 0, yaw: 0 },
            antennas: { right: 0, left: 0 },
        };

        // Signaling
        this._peerId = null;
        this._ws = null;
        this._username = 'local';

        // WebRTC
        this._pc = null;           // RTCPeerConnection
        this._dc = null;           // RTCDataChannel (robot commands)
        this._sessionId = null;
        this._selectedRobotId = null;
        this._pendingIceCandidates = [];

        // Audio
        this._micStream = null;    // MediaStream from getUserMedia
        this._micMuted = true;
        this._audioMuted = true;
        this._micSupported = false; // set after SDP negotiation

        // Timers
        this._latencyMonitorId = null;
        this._stateRefreshInterval = null;

        // startSession() promise plumbing
        this._sessionResolve = null;
        this._sessionReject = null;
        this._iceConnected = false;
        this._dcOpen = false;

        // Set by attachVideo()
        this._videoElement = null;
    }

    // ─── Read-only properties ────────────────────────────────────────────

    /** @returns {"disconnected"|"connected"|"streaming"} */
    get state() { return this._state; }

    /** @returns {Array<{id: string, meta: {name: string}}>} */
    get robots() { return this._robots; }

    /** @returns {{head: {roll:number,pitch:number,yaw:number}, antennas: {right:number,left:number}}} */
    get robotState() { return this._robotState; }

    /** @returns {string|null} */
    get username() { return this._username; }

    /** @returns {boolean} */
    get isAuthenticated() { return true; }

    /** @returns {string} */
    get host() { return this._host; }

    /** @returns {string} */
    get daemonUrl() { return this._daemonUrl; }

    /** @returns {boolean} True if the robot's SDP offered bidirectional audio. */
    get micSupported() { return this._micSupported; }

    /** @returns {boolean} */
    get micMuted() { return this._micMuted; }

    /** @returns {boolean} */
    get audioMuted() { return this._audioMuted; }

    // ─── Auth ────────────────────────────────────────────────────────────

    /** @returns {Promise<boolean>} */
    async authenticate() {
        return true;
    }

    /** No-op in local mode. */
    async login() {
        return true;
    }

    /** Disconnect everything. */
    logout() {
        this.disconnect();
    }

    /** @param {string} host */
    setHost(host) {
        if (this._state !== 'disconnected') throw new Error('Disconnect before changing host');
        this._host = normalizeHost(host);
        this._signalingUrl = buildLocalSignalingUrl(this._host);
        this._daemonUrl = buildLocalDaemonUrl(this._host);
    }

    /** Wake robot motors and trigger wake animation via daemon API. */
    async wakeUp() {
        const wakeRes = await fetch(`${this._daemonUrl}/api/daemon/start?wake_up=true`, {
            method: 'POST',
        });
        if (!wakeRes.ok) throw new Error(`Wake failed (wake_up move): HTTP ${wakeRes.status}`);

        try {
            return await wakeRes.json();
        } catch (_) {
            return null;
        }
    }

    async sleep() {
        const sleepRes = await fetch(`${this._daemonUrl}/api/daemon/stop?goto_sleep=true`, {
            method: 'POST',
        });
        if (!sleepRes.ok) throw new Error(`Sleep failed (goto_sleep move): HTTP ${sleepRes.status}`);

        try {
            return await sleepRes.json();
        } catch (_) {
            return null;
        }
    }

    // ─── Lifecycle ───────────────────────────────────────────────────────

    /**
     * Open local WebSocket signaling connection.
     * Emits "robotsChanged" as robots come and go.
     * @returns {Promise<void>}
     */
    async connect() {
        if (this._state !== 'disconnected') throw new Error('Already connected');
        return new Promise((resolve, reject) => {
            let settled = false;
            const ws = new WebSocket(this._signalingUrl);
            this._ws = ws;

            const fail = (error) => {
                if (settled) return;
                settled = true;
                reject(error instanceof Error ? error : new Error(String(error)));
            };

            ws.onmessage = async (event) => {
                try {
                    const msg = JSON.parse(event.data);
                    await this._handleSignalingMessage(msg);
                    if (!settled && this._state === 'connected') {
                        settled = true;
                        resolve();
                    }
                } catch (e) {
                    this._emit('error', { source: 'signaling', error: e });
                    if (this._state === 'disconnected') fail(e);
                }
            };

            ws.onerror = () => {
                if (!settled) {
                    fail(new Error(`Unable to reach signaling server at ${this._signalingUrl}`));
                }
            };

            ws.onclose = (event) => {
                if (this._ws === ws) this._ws = null;
                const reason = event.reason || 'WebSocket closed';
                const wasConnected = this._state !== 'disconnected';
                this._state = 'disconnected';
                this._peerId = null;
                this._robots = [];
                if (wasConnected) this._emit('disconnected', { reason });
                if (!settled) fail(new Error(reason));
            };
        });
    }

    /**
     * Start a WebRTC session with the given robot.
     * Acquires the microphone (if enabled), negotiates SDP, and waits for
     * both ICE connection and data channel to be ready before resolving.
     * Emits "videoTrack" when the robot's camera stream arrives.
     * Emits "micSupported" once SDP negotiation reveals whether the robot
     * accepts bidirectional audio.
     * @param {string} robotId — one of the ids from the robots list
     * @returns {Promise<void>}
     */
    async startSession(robotId) {
        if (this._state !== 'connected') throw new Error('Not connected');
        this._selectedRobotId = robotId;
        this._iceConnected = false;
        this._dcOpen = false;
        this._micSupported = false;
        this._pendingIceCandidates = [];

        // Acquire mic eagerly so the browser permission prompt appears now,
        // but tracks stay disabled (muted) until the user explicitly unmutes.
        if (this._enableMicrophone) {
            try {
                this._micStream = await navigator.mediaDevices.getUserMedia({ audio: true });
                this._micStream.getAudioTracks().forEach(t => { t.enabled = false; });
                this._micMuted = true;
            } catch (e) {
                console.warn('Microphone not available:', e);
                this._micStream = null;
            }
        }

        this._pc = new RTCPeerConnection();

        return new Promise((resolve, reject) => {
            this._sessionResolve = resolve;
            this._sessionReject = reject;

            this._pc.ontrack = (e) => {
                // Cap the jitter buffer to limit worst-case live-stream latency.
                // jitterBufferTarget (Chrome 113+) sets an upper bound — the
                // browser still adapts within [0, target] based on network
                // conditions, but won't grow beyond this on transient jitter.
                if (e.receiver && 'jitterBufferTarget' in e.receiver) {
                    e.receiver.jitterBufferTarget = 200;  // ms
                }
                if (e.track.kind === 'video') {
                    this._emit('videoTrack', { track: e.track, stream: e.streams[0] });
                }
            };

            this._pc.onicecandidate = async (e) => {
                if (e.candidate) {
                    const ice = e.candidate.toJSON ? e.candidate.toJSON() : {
                        candidate: e.candidate.candidate,
                        sdpMLineIndex: e.candidate.sdpMLineIndex,
                        sdpMid: e.candidate.sdpMid,
                    };
                    if (this._sessionId) {
                        this._sendToServer({ type: 'peer', sessionId: this._sessionId, ice });
                    } else {
                        this._pendingIceCandidates.push(ice);
                    }
                }
            };

            this._pc.oniceconnectionstatechange = () => {
                const s = this._pc?.iceConnectionState;
                if (!s) return;
                if (s === 'connected' || s === 'completed') {
                    this._iceConnected = true;
                    this._checkSessionReady();
                } else if (s === 'failed') {
                    const err = new Error('ICE connection failed');
                    if (this._sessionReject) {
                        this._sessionReject(err);
                        this._sessionResolve = null;
                        this._sessionReject = null;
                    }
                    this._emit('error', { source: 'webrtc', error: err });
                } else if (s === 'disconnected') {
                    this._emit('error', { source: 'webrtc', error: new Error('ICE disconnected') });
                }
            };

            this._pc.ondatachannel = (e) => {
                this._dc = e.channel;
                this._dc.onopen = () => {
                    this._dcOpen = true;
                    this._checkSessionReady();
                };
                this._dc.onmessage = (ev) => this._handleRobotMessage(JSON.parse(ev.data));
            };

            if (!this._sendToServer({ type: 'startSession', peerId: robotId })) {
                const err = new Error('Unable to send startSession to signaling server');
                this._sessionReject(err);
                this._sessionResolve = null;
                this._sessionReject = null;
            }
        });
    }

    /**
     * End the WebRTC session.  Returns to "connected" state so you can
     * startSession() again with the same or a different robot.
     * @returns {Promise<void>}
     */
    async stopSession() {
        if (this._sessionReject) {
            this._sessionReject(new Error('Session stopped'));
            this._sessionResolve = null;
            this._sessionReject = null;
        }

        if (this._stateRefreshInterval) { clearInterval(this._stateRefreshInterval); this._stateRefreshInterval = null; }
        if (this._latencyMonitorId) { clearInterval(this._latencyMonitorId); this._latencyMonitorId = null; }

        if (this._sessionId) {
            this._sendToServer({ type: 'endSession', sessionId: this._sessionId });
        }

        if (this._micStream) { this._micStream.getTracks().forEach(t => t.stop()); this._micStream = null; }
        this._micMuted = true;
        this._micSupported = false;

        if (this._pc) { this._pc.close(); this._pc = null; }
        if (this._dc) { this._dc.close(); this._dc = null; }

        this._sessionId = null;
        this._iceConnected = false;
        this._dcOpen = false;
        this._pendingIceCandidates = [];

        const wasStreaming = this._state === 'streaming';
        if (wasStreaming) {
            this._state = 'connected';
            this._emit('sessionStopped', { reason: 'user' });
        }
    }

    /**
     * Full teardown — abort SSE, close WebRTC.
     * Auth state is preserved (call logout() to also clear credentials).
     */
    disconnect() {
        if (this._ws) {
            const ws = this._ws;
            this._ws = null;
            ws.onclose = null;
            ws.onerror = null;
            ws.onmessage = null;
            try { ws.close(); } catch (_) { /* ignore */ }
        }

        if (this._sessionReject) {
            this._sessionReject(new Error('Disconnected'));
            this._sessionResolve = null;
            this._sessionReject = null;
        }

        if (this._stateRefreshInterval) { clearInterval(this._stateRefreshInterval); this._stateRefreshInterval = null; }
        if (this._latencyMonitorId) { clearInterval(this._latencyMonitorId); this._latencyMonitorId = null; }

        if (this._sessionId) {
            this._sendToServer({ type: 'endSession', sessionId: this._sessionId });
        }

        if (this._micStream) { this._micStream.getTracks().forEach(t => t.stop()); this._micStream = null; }
        if (this._pc) { this._pc.close(); this._pc = null; }
        if (this._dc) { this._dc.close(); this._dc = null; }

        this._sessionId = null;
        this._micMuted = true;
        this._micSupported = false;
        this._iceConnected = false;
        this._dcOpen = false;
        this._peerId = null;
        this._robots = [];
        this._pendingIceCandidates = [];
        this._state = 'disconnected';
        this._emit('disconnected', { reason: 'user' });
    }

    // ─── Commands ────────────────────────────────────────────────────────
    // All return false if the data channel is not open, true if sent.

    /**
     * Set the head orientation.
     * @param {number} roll  — degrees  @param {number} pitch — degrees  @param {number} yaw — degrees
     * @returns {boolean}
     */
    setHeadPose(roll, pitch, yaw) {
        return this._sendCommand({ type: "set_target", head: rpyToMatrix(roll, pitch, yaw).flat() });
    }

    /**
     * Set antenna positions.
     * @param {number} rightDeg  @param {number} leftDeg
     * @returns {boolean}
     */
    setAntennas(rightDeg, leftDeg) {
        return this._sendCommand({ type: "set_antennas", antennas: [degToRad(rightDeg), degToRad(leftDeg)] });
    }

    /**
     * Play a sound file on the robot.
     * @param {string} file — filename available on the robot (e.g. "wake_up.wav")
     * @returns {boolean}
     */
    playSound(file) {
        return this._sendCommand({ type: "play_sound", file });
    }

    /**
     * Send an arbitrary JSON command over the data channel.
     * @param {object} data  @returns {boolean}
     */
    sendRaw(data) {
        return this._sendCommand(data);
    }

    /**
     * Request a state snapshot.  The response arrives as a "state" event.
     * Called automatically every 500 ms while streaming.
     * @returns {boolean}
     */
    requestState() {
        return this._sendCommand({ type: "get_state" });
    }

    // ─── Audio ───────────────────────────────────────────────────────────

    /**
     * Mute/unmute the robot's audio playback (speaker) locally.
     * Audio is muted by default — browsers require a user gesture to unmute.
     * @param {boolean} muted
     */
    setAudioMuted(muted) {
        this._audioMuted = muted;
        if (this._videoElement) {
            this._videoElement.muted = muted;
            // When unmuting, flush the stale audio buffer by toggling the
            // audio track off/on.  This forces the browser to resync to the
            // live edge instead of playing seconds-old buffered audio.
            if (!muted && this._videoElement.srcObject) {
                for (const t of this._videoElement.srcObject.getAudioTracks()) {
                    t.enabled = false;
                    t.enabled = true;
                }
            }
        }
    }

    /**
     * Mute/unmute your microphone.  Only works if micSupported is true.
     * Mic is muted by default even after acquisition.
     * @param {boolean} muted
     */
    setMicMuted(muted) {
        this._micMuted = muted;
        if (this._micStream) {
            this._micStream.getAudioTracks().forEach(t => { t.enabled = !muted; });
        }
    }

    // ─── Video helper ────────────────────────────────────────────────────

    /**
     * Bind a `<video>` element to this robot's stream.
     * Call before startSession().  Sets srcObject when the video track arrives,
     * applies audio mute state, and runs a latency monitor that snaps to the
     * live edge if the buffer grows > 0.5 s.
     *
     * @param {HTMLVideoElement} videoElement
     * @returns {() => void} cleanup function — call to detach video and stop monitoring
     */
    attachVideo(videoElement) {
        this._videoElement = videoElement;
        videoElement.muted = this._audioMuted;

        const onVideoTrack = (e) => {
            videoElement.srcObject = e.detail.stream;
            videoElement.playsInline = true;
            if ('requestVideoFrameCallback' in videoElement) {
                this._startLatencyMonitor(videoElement);
            }
        };

        const onSessionStopped = () => { videoElement.srcObject = null; };

        this.addEventListener('videoTrack', onVideoTrack);
        this.addEventListener('sessionStopped', onSessionStopped);

        return () => {
            this.removeEventListener('videoTrack', onVideoTrack);
            this.removeEventListener('sessionStopped', onSessionStopped);
            if (this._latencyMonitorId) { clearInterval(this._latencyMonitorId); this._latencyMonitorId = null; }
            videoElement.srcObject = null;
            this._videoElement = null;
        };
    }

    // ─── Private ─────────────────────────────────────────────────────────

    _emit(name, detail) {
        this.dispatchEvent(new CustomEvent(name, { detail }));
    }

    async _sendToServer(message) {
        try {
            if (!this._ws || this._ws.readyState !== WebSocket.OPEN) return false;
            this._ws.send(JSON.stringify(message));
            return true;
        } catch (e) {
            console.error('Send error:', e);
            return false;
        }
    }

    _sendCommand(cmd) {
        if (!this._dc || this._dc.readyState !== 'open') return false;
        this._dc.send(JSON.stringify(cmd));
        return true;
    }

    /** Resolves the startSession() promise once both ICE and datachannel are ready. */
    _checkSessionReady() {
        if (this._iceConnected && this._dcOpen && this._sessionResolve) {
            this._state = 'streaming';
            this.requestState();
            this._stateRefreshInterval = setInterval(() => this.requestState(), 500);
            this._emit('streaming', { sessionId: this._sessionId, robotId: this._selectedRobotId });
            this._sessionResolve();
            this._sessionResolve = null;
            this._sessionReject = null;
        }
    }

    async _handleSignalingMessage(msg) {
        switch (msg.type) {
            case 'welcome':
                this._peerId = msg.peerId;
                this._sendToServer({
                    type: 'setPeerStatus',
                    roles: ['listener'],
                    meta: { name: 'Reachy Mini Local Controller' },
                });
                break;
            case 'list':
                this._robots = msg.producers || [];
                this._emit('robotsChanged', { robots: this._robots });
                break;
            case 'peerStatusChanged': {
                if (msg.peerId === this._peerId && Array.isArray(msg.roles) && msg.roles.includes('listener')) {
                    if (this._state !== 'connected') {
                        this._state = 'connected';
                        this._emit('connected', { peerId: msg.peerId });
                    }
                }
                this._sendToServer({ type: 'list' });
                break;
            }
            case 'sessionStarted':
                this._sessionId = msg.sessionId;
                for (const ice of this._pendingIceCandidates) {
                    this._sendToServer({ type: 'peer', sessionId: this._sessionId, ice });
                }
                this._pendingIceCandidates = [];
                break;
            case 'peer':
                this._handlePeerMessage(msg);
                break;
        }
    }

    async _handlePeerMessage(msg) {
        if (!this._pc) return;
        try {
            if (msg.sdp) {
                const sdp = normalizeSessionDescription(msg.sdp);
                if (!sdp) throw new Error('Invalid SDP payload');
                if (sdp.type === 'offer') {
                    const supportsMic = sdpHasAudioSendRecv(sdp.sdp);
                    this._micSupported = supportsMic;
                    this._emit('micSupported', { supported: supportsMic });

                    // Mic track must be added BEFORE setRemoteDescription so the
                    // generated answer naturally includes sendrecv for audio.
                    if (supportsMic && this._micStream) {
                        for (const track of this._micStream.getAudioTracks()) {
                            this._pc.addTrack(track, this._micStream);
                        }
                    }

                    await this._pc.setRemoteDescription(new RTCSessionDescription(sdp));
                    const answer = await this._pc.createAnswer();
                    await this._pc.setLocalDescription(answer);
                    await this._sendToServer({
                        type: 'peer',
                        sessionId: this._sessionId,
                        sdp: { type: 'answer', sdp: answer.sdp },
                    });
                } else {
                    const supportsMic = sdpSupportsMicrophone(sdp.sdp);
                    this._micSupported = supportsMic;
                    this._emit('micSupported', { supported: supportsMic });
                    await this._pc.setRemoteDescription(new RTCSessionDescription(sdp));
                }
            }
            if (msg.ice) {
                await this._pc.addIceCandidate(new RTCIceCandidate(msg.ice));
            }
        } catch (e) {
            console.error('WebRTC error:', e);
            this._emit('error', { source: 'webrtc', error: e });
        }
    }

    /** Parse robot state (rotation matrix + radians) into degrees and emit. */
    _handleRobotMessage(data) {
        if (data.state) {
            const s = data.state;
            if (s.head_pose) this._robotState.head = matrixToRpy(s.head_pose);
            if (s.antennas) {
                this._robotState.antennas = {
                    right: radToDeg(s.antennas[0]),
                    left:  radToDeg(s.antennas[1]),
                };
            }
            this._emit('state', { ...this._robotState });
        }
        if (data.error) {
            this._emit('error', { source: 'robot', error: data.error });
        }
    }

    /** Snap video playback to live edge if buffered lag exceeds 0.5 s. */
    _startLatencyMonitor(video) {
        if (this._latencyMonitorId) clearInterval(this._latencyMonitorId);
        this._latencyMonitorId = setInterval(() => {
            if (!video.srcObject || video.paused) return;
            const buf = video.buffered;
            if (buf.length > 0) {
                const end = buf.end(buf.length - 1);
                const lag = end - video.currentTime;
                if (lag > 0.5) {
                    console.log(`Latency correction: was ${lag.toFixed(2)}s behind`);
                    video.currentTime = end - 0.1;
                }
            }
        }, 2000);
    }
}

export default ReachyMini;