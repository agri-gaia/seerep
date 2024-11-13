# Tutorials Overviews

The tutorials provide a starting point on how you can use SEEREP. Currently, the
following topics are covered:

- Creating and retrieving projects
- Sending and querying images
- Projects and querying in other spatial reference systems
- Writing python examples
- Writing python integration tests

Before running any of the tutorials, make sure that
you have a  **running SEEREP instance available**.

If you're not familiar with flatbuffers, it is highly recommended to look at the
[tutorial](https://flatbuffers.dev/flatbuffers_guide_tutorial.html) first.
Click on the python radio button to see the language specific tutorial.

## Local Instance

To start SEEREP locally use `STRG+SHIFT+D` to open the Run & Debug Menu in
Vs-Code, select `seerep server` and press run. Now a terminal should open and
print the following info messages:

```bash
[2024-07-16 14:41:27.416289]<info>: Initialized logging
[2024-07-16 14:41:27.416806]<info>: The used logging folder is: /seerep/seerep-data/debug/log/
[2024-07-16 14:41:27.416847]<info>: Current timezone: CET
[2024-07-16 14:41:27.416879]<info>: SEEREP version: v0.2.7-25-g48531131
[2024-07-16 14:41:27.416918]<info>: The used data folder is: /seerep/seerep-data/debug/
[2024-07-16 14:41:27.417739]<info>: Addded Protocol Buffers gRPC services
[2024-07-16 14:41:27.418177]<info>: Added Flatbuffers gRPC services
[2024-07-16 14:41:27.423797]<info>: Serving gRPC Server on "[::]:9090"
```
