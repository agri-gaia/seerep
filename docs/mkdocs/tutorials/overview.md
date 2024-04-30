# Tutorials Overviews

The tutorials provide a starting point on how you can use SEEREP. Currently, the following topics are covered:

- Creating and Retrieving projects
- Sending and Querying images
- Writing python examples
- Writing python integration tests

Before running any of the tutorials, make sure that
you have a  **running SEEREP instance available**.

## Local Instance

To start SEEREP locally use `STRG+SHIFT+D` to open the Run & Debug Menu in Vs-Code, select `seerep server` and press run.
Now a terminal should open and print the following info messages:

```bash
Starting seerep server
[2022-08-01 13:50:35.765427]<info>: The used logging folder is: /seerep/seerep-data/log/
[2022-08-01 13:50:35.765575]<info>: The used data folder is: /seerep/seerep-data/
[2022-08-01 13:50:35.765801]<info>: add the protobuf gRPC services...
[2022-08-01 13:50:35.765860]<info>: add the flatbuffer gRPC services...
[2022-08-01 13:50:35.767787]<info>: serving gRPC Server on "[::]:9090"...
```
