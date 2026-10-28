# ACT / SO-101 calibration helpers

This is NOT a calibration result. These tools only help investigate the training convention and fit coordinate conversions from actual paired, physically equivalent poses.

1. Inspect model and dataset metadata (inside the `lerobot` container):

```bash
docker compose exec lerobot python /workspace/inspect_reference.py
```

2. Copy `joint_correspondences.template.json` to `workspace/joint_correspondences.json` and manually measure at least three `fit` pairs plus two independent `validate` pairs. Every entry must have `sim: [6 numbers]` and `policy: [6 numbers]` in the exact `joint_names` order, representing the **same physical configuration**. Sample shape (illustrative only, **not real measurements**):

```json
{"sim": [0,0,0,0,0,0], "policy": [0,0,0,0,0,0]}
```

3. Fit inside `lerobot` where `/workspace` is read-only and `/outputs` is writable:

```bash
docker compose exec lerobot python /workspace/fit_calibration.py /workspace/joint_correspondences.json --output /outputs/calibration.json
```

4. Inspect per-joint residuals and manually verify the kinematics, camera view and gripper at five or more known configurations. The output keeps `calibration_confirmed` false. Never enable control solely on the basis of an affine fit.

5. If the output is verified, `run_act.py` can read `--calibration /outputs/calibration.json` but it is still necessary to check that the robot's geometry and task distribution match the original SO-101 training environment. The demo arm is not a faithful SO-101 model.
