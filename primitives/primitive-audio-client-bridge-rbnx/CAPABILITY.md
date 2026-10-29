---
name: Client audio bridge
description: Route Robonix microphone and speaker streams through a connected robonix-client.
---

# Client audio bridge primitive

Use this provider when the microphone and speaker belong to the computer
running `robonix-client`, not to the robot host. The default reverse transport
lets the Client discover the robot endpoint through Atlas and connect without
storing the Client IP in the deployment.

The provider exposes the standard audio streaming and device-routing contracts,
plus bridge connection information for Client discovery and diagnostics.
