# Robonix Client Audio Bridge Primitive

This package exposes a `robonix-client` microphone and speaker through the
standard Robonix audio contracts. It runs on the robot host and is independent
of Webots or any specific robot deployment.

## How the connection works

The default `reverse` transport makes the robot listen on TCP port `60002`.
`robonix-client` discovers that endpoint through Atlas and opens the outbound
WebSocket connection. The robot manifest therefore does not need the Client's
IP address.

```text
robonix-client microphone/speaker
              |
              | outbound WebSocket
              v
audio_client_bridge on the robot
              |
              | standard Robonix audio contracts
              v
       Liaison and Speech
```

## Deployment

Add the package to a robot deployment manifest:

```yaml
primitive:
  - name: audio_client_bridge
    url: https://github.com/syswonder/primitive-audio-client-bridge-rbnx
    branch: main
    config:
      transport: reverse
      listen_host: 0.0.0.0
      listen_port: 60002
```

Then build and boot the deployment:

```bash
rbnx build
rbnx boot
```

The build creates an isolated package environment before code generation,
installs a pinned protobuf/gRPC compiler and runtime into it, and directs
`rbnx codegen` to that same interpreter. It then imports every generated
`*_pb2.py` and `*_pb2_grpc.py` module. Package startup reuses this environment,
so generated bindings are never validated with a different Python installation
than the one that runs the bridge.

Start the Client on the computer that owns the microphone and speaker:

```bash
robonix-client --robot-host <robot-ip>
```

Open **Audio**, choose `audio_client_bridge` for the input and output routes,
select the local devices, and apply the route. The Client connects to the
reverse endpoint advertised by the primitive.

The full runtime configuration, including the deprecated client-owned server
transport, is documented in [`config.spec`](config.spec).

## Capabilities

| Contract | Purpose |
| --- | --- |
| `robonix/primitive/audio/mic` | Streams Client microphone PCM to Robonix. |
| `robonix/primitive/audio/speaker` | Plays Robonix PCM on the Client. |
| `robonix/primitive/audio/list_devices` | Lists Client devices and the current route. |
| `robonix/primitive/audio/select_device` | Changes the Client input or output device. |
| `robonix/primitive/audio/bridge_info` | Reports the reverse endpoint and connection state. |
| `robonix/primitive/audio/driver` | Provides the standard lifecycle interface. |

Audio frames are mono, 16-bit PCM at 16 kHz. The reverse WebSocket currently
has no TLS or application-level authentication; expose it only on a trusted
network, Tailscale, or an authenticated tunnel.

## Development checks

```bash
python3 -m unittest discover -s tests -v
rbnx build
```

The `client_audio_server` directory is retained for compatibility with the
deprecated client-owned server transport. New deployments should use the
reverse transport managed by `robonix-client`.
