# Installation Guide

These instructions were verified on a fresh **Ubuntu 24.04.5 LTS**
system with **GCC/G++ 13.3.0** and **Python 3.12.3**. Other recent
Debian-based distributions with similarly recent GCC/Python versions
should work with the same steps, but have not been separately verified.

## 1. Required system packages

```bash
sudo apt update
sudo apt install -y build-essential python3 python3-dev pkg-config \
    libxml2-dev libsqlite3-dev git
```

- `build-essential` — provides GCC/G++ and `make`.
- `python3` — required by the `waf` build system (version 3 only; see
  the Python 3 compatibility note below).
- `pkg-config`, `libxml2-dev`, `libsqlite3-dev` — enable the `XmlIo` and
  `SqliteDataOutput` optional ns-3 features. Not required for the core
  build; `waf configure` will simply report them as unavailable if
  skipped.

### Optional packages

These enable optional ns-3 features not required by the routing
scenarios in this repository:

```bash
sudo apt install -y libgtk-3-dev libgsl-dev libgcrypt-dev doxygen valgrind
```

- `libgtk-3-dev` → `GtkConfigStore` (this project's verified build used
  GTK 2, which `waf configure` reported as unavailable on a GTK-3-only
  system; this feature is not used by the routing scenarios and can be
  left disabled).
- `libgsl-dev` → `GSL` optional feature (not used by the routing
  scenarios).
- `doxygen` → only needed for `./waf doxygen` (API documentation
  generation), not for building or running simulations.

Python bindings (`PyViz` / the `python` optional feature) require
`pybindgen` and a working `python-config`; this project's verified
build did not enable them, and they are not required to build or run
any of the routing scenarios.

## 2. Clone the repository

```bash
git clone https://github.com/praveenreddy4512/wifi-halow-mesh-routing-ns3.git
cd wifi-halow-mesh-routing-ns3/ns-3
```

All build and run commands below are executed from inside the `ns-3/`
directory — that is where `waf` and `wscript` live.

## 3. Python 3 compatibility note (important)

This repository bundles `waf` version 1.8.19, originally written for
Python 2 / early Python 3. Running it with Python 3.12 or newer fails
with `ModuleNotFoundError: No module named 'imp'` unless patched,
because Python 3.12 removed the long-deprecated `imp` module that this
waf version's internal library (`waflib`) imports.

**This has already been fixed in this repository.** The directory
`ns-3/.waf3-1.8.19-b1fc8f7baef51bd2db4c2971909a568d/` is `waf`'s
self-extracted internal runtime library, pre-patched with four minimal
changes (removing the `imp` import and `imp.new_module`/`imp.get_tag`
calls, the deprecated `'rU'` file-open mode, and a `raise StopIteration`
inside a generator, which Python 3.7+ no longer permits). If you ever
see `waf` try to re-extract this directory (for example, after deleting
it) and the `imp`/`StopIteration` errors return, see
`docs/SOURCE_CODE_STRUCTURE.md` for the exact four changes to reapply,
or copy the directory back from version control.

## 4. Configure

```bash
cd ns-3
python3 ./waf configure
```

Expected output ends with a feature summary and `'configure' finished
successfully`. A typical verified summary on Ubuntu 24.04 shows Python
Bindings, Examples, and Tests as **not enabled** (these are optional
ns-3 features with additional dependencies not installed by default,
per step 1) and Real Time Simulator, Threading Primitives, Tap Bridge,
and Emulation FdNetDevice as **enabled**.

If you want the ns-3 example binaries or test suite built as well:

```bash
python3 ./waf configure --enable-examples --enable-tests
```

This was not the configuration used for this repository's own build
verification; if you enable it, expect a longer build and verify it
separately.

## 5. Build

```bash
python3 ./waf build
```

A full build compiles on the order of 1,700+ translation units across
all bundled ns-3 modules (most of which are unmodified upstream ns-3
modules required by the dependency graph, not specific to this
project's routing work) and will take several minutes to over an hour
depending on CPU. Expect many `-Wdeprecated-declarations` warnings from
old ns-3 code built against a modern GCC (e.g. `std::unary_function`
deprecation) — these are warnings, not errors, and do not indicate a
build failure.

## 6. Verify the build

```bash
python3 ./waf --run "scratch/five-floor-building-mesh --protocol=aodv --l2Mode=mesh11s-l3 --validationNodes=4 --pcap=false --flowmon=false"
```

A successful run prints simulation progress to stdout and exits with
status 0. If it does, your environment can build and run every
scenario in this repository.

You can also run the bundled validation script:

```bash
bash scripts/validate-mesh-stack.sh
```

## 7. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `ModuleNotFoundError: No module named 'imp'` | A fresh, unpatched `.waf3-1.8.19-*/` directory was extracted (e.g., you deleted the bundled one). | Restore the bundled, patched directory from version control, or reapply the four changes in `docs/SOURCE_CODE_STRUCTURE.md`. |
| `SyntaxError` inside `waflib/Tools/python.py` | A manual edit to that file introduced a quoting error (seen during this repository's own preparation — see history for the fix). | Verify the line matches the patched version shipped in this repository. |
| `boost headers not found` / `click location not found` / `NSC location not found` | Optional ns-3 features (OpenFlow, Click, Network Simulation Cradle); none are used by this project's scenarios. | Ignore — `waf configure` reports these as simply "not enabled," not as errors. |
| Build is very slow | A full `./waf build` compiles every bundled ns-3 module, most of which are standard upstream modules required by the build graph, not just the routing modules this project modifies. | Expected; see step 5. There is no documented way to build only a subset of modules with this ns-3 snapshot's waf configuration without risking missing dependencies. |
| `Waf: The wscript in '...' is unreadable` / `ModuleNotFoundError: No module named 'utils'` | Running `waf` from the wrong directory, or `utils.py` / `utils/` missing. | Run all commands from inside `ns-3/`; confirm `ns-3/utils.py` and `ns-3/utils/` exist. |

## 8. Running the full multi-protocol scenario

```bash
python3 ./waf --run "scratch/five-floor-building-mesh --protocol=aodvv2 --l2Mode=mesh11s-l3 --validationNodes=20 --enableLinkQualityMetric=true"
```

See the root [README.md](README.md#13-configuration-options) for the
full configuration-option table and
[docs/ROUTING_PROTOCOLS.md](docs/ROUTING_PROTOCOLS.md) for how protocol
selection works.
