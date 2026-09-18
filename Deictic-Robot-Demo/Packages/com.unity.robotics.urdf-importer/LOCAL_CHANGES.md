# Embedded package compatibility changes

The four native Assimp plugin `.meta` files now restrict each binary to its actual desktop OS/architecture and disable Android. The supplied 0.5.2 metadata had duplicate Any-platform entries that caused Unity 6000.6 to include both Windows `assimp.dll` files in the Android build, failing plugin collection. GUIDs and binaries are unchanged.

K1 meshes are imported/generated in the editor and serialized into the supplied prefab. This demo does not attempt native Assimp import on the Quest device.
