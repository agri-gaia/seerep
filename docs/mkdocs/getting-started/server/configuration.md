# Configure the server

When starting the server some options to configure the server are available. The configuration is possible via the
command line, via a config file or via environment variables.

## Command line

For the command line the following options are available. All of them are optional and have default values. The latest
list of options can always be retrieved by starting the server with the `--help` argument.

```bash
Generic options:
  -v [ --version ]                      print version string
  --help                                produce help message
  -c [ --config ] arg                   name of a file of a configuration.

Configuration:
  -D [ --data-folder ] arg (=/seerep/devel/bin)
                                        data folder
  -L [ --log-path ] arg                 log path
  --log-level arg (=info)               log-level [trace, debug, info, warning,
                                        error, fatal]
  -p [ --port ] arg (=9090)             gRPC port
```

## Config file

The configuration options of the command line options can also be set via a config file. The path and name of the config
file has to be given to the server using the `--config` option.

An example config file:

```python
data-folder = /seerep/seerep-data/ #defaulting to work dir
log-path = /seerep/seerep-data/log/ #file logging disabled if not set
log-level = info
port = 9090
```

## Environment Variables

If one of the following environment variables is set, it will be parsed and set as the command line equivalent.

| Environment variable  | command line equivalent   |
|---                    |---                        |
| SEEREP_DATA_FOLDER    | --data-folder             |
| SEEREP_LOG_PATH       | --log-path                |
| SEEREP_LOG_LEVEL      | --log-level               |
| SEEREP_PORT           | --port                    |
