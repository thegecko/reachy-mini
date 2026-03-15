# Reachy Mini local controller

This version connects directly to a local Reachy Mini instead of using Hugging Face auth.

## Run locally

Serve the folder with any static file server, then open the page in a browser.

Examples:

- `python3 -m http.server 8080`
- `npx serve .`

Then:

1. Open the app in your browser.
2. Enter your robot hostname or IP, for example `reachy-mini.local` or `192.168.1.18`.
3. Click **Use local robot**.
4. Click **Connect** to discover local producers and start the WebRTC session.

The app expects the Reachy Mini local signaling server on port `8443`.
