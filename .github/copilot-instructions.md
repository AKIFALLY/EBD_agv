# RosAGV AI Coding Assistant Instructions

## 1. High-Level Architecture

- **Dual-Environment System:** The project is split into two Docker environments:
    1.  **AGV On-board System (`rosagv` container):** For real-time AGV control. Managed by `docker-compose.yml`.
    2.  **AGVC Management System (`agvc_server` container):** For fleet management, database, and web UI. Managed by `docker-compose.agvc.yml`.
- **ROS 2 with Zenoh:** We use ROS 2 Jazzy with `rmw_zenoh_cpp` for communication. The Zenoh router configuration is in `app/routerconfig.json5`.
- **Python Virtual Environment:** A separate Python virtual environment exists at `/opt/pyvenv_env` for non-ROS packages like FastAPI and SQLAlchemy.

## 2. Key Development Workflows

- **Environment Setup:** Inside a container, always start by sourcing the environment script: `source /app/setup.bash`. This script provides essential aliases and functions.
- **Building Workspaces:** Use `colcon build` to compile workspaces. The alias `build_all` will build everything.
- **Running Tests:** Use `colcon test` to run tests. The alias `test_all` is also available.
- **Python Usage:**
    - Always use `python3` and `pip3`.
    - For packages in the virtual environment (e.g., `fastapi`), use `/opt/pyvenv_env/bin/python3` and `/opt/pyvenv_env/bin/pip3`.

## 3. Project-Specific Conventions

- **READMEs are Mandatory Reading:** Before editing any workspace, you **must** read its `README.md`. It contains critical information about dependencies, startup status, and functionality.
- **Documentation-First:** Any change to code that alters functionality, dependencies, or usage requires a corresponding update to the relevant `README.md` file.
- **Status Icons:** Pay attention to the status icons in READMEs (e.g., ✅ for active, ⚠️ for manual, ❌ for deprecated) to understand the current state of a workspace or feature.

## 4. Key Integration Points

- **PLC Communication:** `plc_proxy_ws` and `keyence_plc_ws` are the gateways to PLC hardware.
- **Database Access:** All database operations are handled through the ROS 2 services provided by `db_proxy_ws`. Do not connect to the database directly from other nodes.
- **Web API:** The `web_api_ws` provides a FastAPI-based REST API. This is the primary entry point for external systems and the front-end. See details below.
- **KUKA Fleet Integration:** The `kuka_fleet_ws` and `wcs_ws` are responsible for communicating with the KUKA Fleet management system.

### Web API (`web_api_ws`) Specifics

- **Tech Stack:** This workspace uses **FastAPI**, **Uvicorn**, and **Pydantic** from the `/opt/pyvenv_env` virtual environment.
- **Modular Structure:** It contains three distinct services:
    - `web_api`: The core RESTful API backend.
    - `agvcui`: The comprehensive AGV management UI.
    - `opui`: The simplified operator UI.
- **Router-Based Architecture:** The `web_api` service uses a router pattern. Key business logic is encapsulated in `src/web_api/web_api/routers/`.
    - **Example:** `routers/door.py` handles door control by importing and using `DoorLogic` from the `ecs_ws` workspace. This replaced the old MQTT-based system.
    - Other routers integrate with `plc_proxy_ws` (`plc.py`) and `rcs_ws` (`traffic.py`).
- **Data Models:** API request and response models are defined using Pydantic's `BaseModel`.
- **Frontend:** The `agvcui` and `opui` frontends are built with **Jinja2** templates and serve static assets (JS/CSS) from their respective `static/` directories. They use **Socket.IO** for real-time updates.

## 5. How to Approach a Task

1.  **Identify the Environment:** Determine if the task relates to the AGV on-board system or the AGVC management system.
2.  **Consult the README:** Locate the relevant workspace(s) and read their `README.md` files thoroughly.
3.  **Check Dependencies:** Note whether the workspace uses system ROS 2 packages or packages from the Python virtual environment.
4.  **Implement Changes:** Make your code changes, following the existing style and conventions.
5.  **Update Documentation:** Update the `README.md` to reflect your changes. This is a required step.
6.  **Build and Test:** Use `colcon build` and `colcon test` to verify your changes.