# Source and license notices

The [MIT License](LICENSE) covers the workspace and bundled testbed code unless
a package or file carries a separate license.

- `testbed_broadcasters`, `testbed_hardware_interfaces` and
  `testbed_joint_rcm_controller` use the [Apache License 2.0](LICENSES/Apache-2.0.txt).
- The optional Lambda driver uses the [GNU General Public License version 3](LICENSES/GPL-3.0.txt).
- The [UR control launch file](src/robots_controllers/controllers/testbed_launchers/launch/ur_control.launch.py)
  retains the PickNik copyright and BSD terms in its source.

The packages under `src/` are based on these testbed revisions.

| Source | Revision |
| --- | --- |
| Robot controllers | `2178b6a5030d34de5d63f628c4d6e4c4acaa3d5c` |
| Input devices | `6e412ec5542ea8dd215ed96e2ef69fd180507f2e` |
| Messages and services | `6192ce8eb593574d1af9ec766dbf540e2f12ae7f` |

The camera mount example and recorder profiles are based on the MIT-licensed
Sensorium integration. Dependencies fetched via `sources.repos` carry their
own license files.
