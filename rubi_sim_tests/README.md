# RUBI simulation tests

This package contains read-only model inspection tools for the user-approved
canonical RUBI 6-DoF encoder and policy. Controller execution lives in
`rubi_control_core` and the simulator adapters, not in this package.

The package vendors only the ONNX Runtime headers, CPU shared library, license,
and the two canonical ONNX files needed for reproducible inspection. The
rejected 280-to-3 / 35-to-8 pair is quarantined under
`models/rejected_unknown_8action` and is excluded from install rules.
