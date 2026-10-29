# Piper robot description

`piper.urdf` describes the Piper arm and parallel gripper. The root
[soma.yaml](../soma.yaml) and IK service use this model.

Mesh references use the standard ROS package URI
`package://piper_description/meshes/`. The corresponding STL files are included
in `primitives/primitive-agilex-piper-description-rbnx/src/piper_description/meshes/`
and installed by the bundled description package. Keeping one copy preserves
the same geometry for ROS visualization and the deployment model.

The deployment photograph is [assets/promo/01.jpg](../assets/promo/01.jpg).
