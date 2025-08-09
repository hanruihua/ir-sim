# Contributing to IR-SIM

Thank you for your interest in improving IR-SIM! Whether you’re fixing a typo, adding a new feature, or writing documentation, your contribution is welcome by opening an issue or a pull request to the `main` branch. Below are a few guidelines to help you get started.

---

## What You Can Contribute

- **Source Code**  
  - New features (sensors, controllers, kinematic models)  
  - Bug fixes and performance improvements  

- **Documentation**  
  - Fix typos, clarify API descriptions  
  - Add new tutorials or usage guides  
  - Improve examples under [`doc/`](https://ir-sim.readthedocs.io/en/stable/)  

- **Usage Examples & Tests**  
  - Extend or polish the [`irsim/usage/`](https://github.com/hanruihua/ir-sim/tree/main/irsim/usage) directory  
  - Add unit tests for new features

- **Discussion & Design**  
  - Propose architectural changes  
  - Provide feedback on roadmap or UX  

---

## Code Style

We use **[Ruff](https://docs.astral.sh/ruff/)** to format and lint Python code. Before opening a PR, run:

```bash
pip install ruff
ruff check
```

To enable automatic checks on each commit, install the pre-commit hook:

```bash
pip install pre-commit
pre-commit install
```

---

## Documentation

IR-SIM’s documentation lives in the `doc/` directory and uses the [PyData Sphinx Theme](https://pydata-sphinx-theme.readthedocs.io/en/stable/). To build the docs locally:

```bash
cd doc
make html
```

- New tutorials or guides belong in `doc/` or under `irsim/usage/`  
- The API reference is auto-generated from docstrings  

---

## Adding New Features

Current support of sensors, behaviors, and robot kinematics models can be seen in the [support](https://github.com/hanruihua/ir-sim?tab=readme-ov-file#support).

New features to support more platforms and scenarios to make IR-SIM more versatile are welcome. Please open an issue to discuss the design. If you prefer, you can also email `hanrh@connect.hku.hk` for a design discussion. 

---

## Testing

We use **pytest** for our test suite. To run all tests:

```bash
pytest
```

Please add tests for any new functionality or bug fix.

---


Thanks again for helping make IR-SIM better!