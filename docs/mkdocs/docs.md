# Docs

The documentation is published via GitHub Pages. The responsible workflow
combines `Doxygen` and `MkDocs` into one page, so it is more convenient to ues.
MkDocs focusses on higher level concepts like the installation process and a
package overview, while Doxygen ist used for code documentation. If you want to
work on the documentation locally i.e for a PR follow the steps below. Locally the two
documentation systems must be run individually.

## Dependencies

**If you are not using the SEEREP development container**, you need to have `doxygen`
and `MkDocs-Material` installed, use the commands below:

```bash
pip3 mkdocs-material
sudo apt install doxygen
```

## MkDocs

To run MkDocs locally switch into the main directory of SEEREP, where the
`mkdocs.yml` is located. Then use `mkdocs serve` to build
and deploy MkDocs on a local http-server. The page should then be available
under [http://127.0.0.1:8000/](http://127.0.0.1:8000/).

## Doxygen

To create the Doxygen output locally switch into the main directory of SEEREP, where the
`Doxyfile` is located and run `doxygen Doxyfile`. Now an `html/` folder should
be in the same directory.

If you are **not working in the development container** you can simply open the
`index.html` with your browser of choice. Otherwise, switch into the `html`
folder and use `python3 -m http.server` to start a local web server who serves the
content. The page should be available under the default
[http://0.0.0.0:8000/](http://0.0.0.0:8000/) address.

**If you want to run [MkDocs](#mkdocs) and [Doxygen](#doxygen) at the same time**
you need to provide a different port, use `python3 -m
http.server 8002` instead.
