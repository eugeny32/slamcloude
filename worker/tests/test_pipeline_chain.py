"""Chain construction is pure (no broker/DB) — verify order and wiring."""

from app.models import PIPELINE_ORDER, PipelineStep
from pipeline.tasks import build_pipeline, intermediate_key


def test_chain_covers_all_steps_in_canonical_order() -> None:
    sig = build_pipeline("scan-123")
    tasks = list(sig.tasks)
    assert [t.name for t in tasks] == ["pipeline.step"] * len(PIPELINE_ORDER)
    assert [t.args for t in tasks] == [
        ("scan-123", step.value) for step in PIPELINE_ORDER
    ]


def test_chain_signatures_are_immutable() -> None:
    # .si() — a failed/skipped step's return value must not leak into the next.
    assert all(t.immutable for t in build_pipeline("x").tasks)


def test_intermediate_keys_are_scoped_to_scan_and_step() -> None:
    keys = {intermediate_key("abc", step) for step in PipelineStep}
    assert len(keys) == len(PipelineStep)  # no collisions between steps
    assert all(k.startswith("abc/intermediate/") for k in keys)
