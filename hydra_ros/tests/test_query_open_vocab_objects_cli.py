"""Tests for the live open-vocabulary object-query CLI."""

from __future__ import annotations

from importlib.machinery import SourceFileLoader
from importlib.util import module_from_spec, spec_from_loader
from pathlib import Path

import pytest
from sensor_msgs.msg import CompressedImage


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "app" / "query_open_vocab_objects"
LOADER = SourceFileLoader("query_open_vocab_objects_cli", str(SCRIPT_PATH))
SPEC = spec_from_loader(LOADER.name, LOADER)
assert SPEC is not None
query_mod = module_from_spec(SPEC)
LOADER.exec_module(query_mod)


@pytest.mark.parametrize("option", ["--text", "--query"])
def test_parser_accepts_text_and_legacy_query_alias(option):
    args = query_mod.build_parser().parse_args([option, "armchair"])

    assert args.query_inputs == [("text", "armchair")]


def test_parser_accepts_image_query():
    args = query_mod.build_parser().parse_args(["--image", "/tmp/plant.jpg"])

    assert args.query_inputs == [("image", "/tmp/plant.jpg")]


def test_parser_preserves_mixed_text_image_order():
    args = query_mod.build_parser().parse_args(
        [
            "--text",
            "stairs",
            "--query",
            "red car",
            "--image",
            "/tmp/barrel.png",
        ]
    )

    assert args.query_inputs == [
        ("text", "stairs"),
        ("text", "red car"),
        ("image", "/tmp/barrel.png"),
    ]


def test_parser_keeps_query_feature_mode():
    args = query_mod.build_parser().parse_args(["--query-feature", "/tmp/query.json"])

    assert args.query_inputs == [("feature", "/tmp/query.json")]


def test_parser_accumulates_multiple_text_queries_and_top_ks():
    args = query_mod.build_parser().parse_args(
        [
            "--query",
            "armchair",
            "--top-k",
            "3",
            "--text",
            "table",
            "--top-k",
            "2",
        ]
    )

    assert args.query_inputs == [("text", "armchair"), ("text", "table")]
    assert query_mod.resolve_top_ks(len(args.query_inputs), args.top_k) == [3, 2]


def test_one_top_k_is_shared_by_all_queries():
    assert query_mod.resolve_top_ks(3, [4]) == [4, 4, 4]


def test_top_k_count_must_match_query_count():
    with pytest.raises(ValueError, match="one --top-k value"):
        query_mod.resolve_top_ks(3, [1, 2])


def test_min_mesh_vertices_preserves_existing_default_when_unset():
    args = query_mod.build_parser().parse_args(["--query", "chair"])

    assert args.min_mesh_vertices is None
    assert query_mod.resolve_min_mesh_vertices(args.min_mesh_vertices) == 20


def test_min_mesh_vertices_accepts_explicit_zero_and_positive_value():
    assert query_mod.resolve_min_mesh_vertices(0) == 0
    assert query_mod.resolve_min_mesh_vertices(37) == 37


def test_main_sends_multi_query_requests_with_accumulation(monkeypatch):
    requests = []

    class FakeFuture:
        def __init__(self):
            self._response = query_mod.QueryOpenVocabObjects.Response()
            self._response.success = True
            self._response.message = "ok"

        def done(self):
            return True

        def result(self):
            return self._response

    class FakeClient:
        def wait_for_service(self, timeout_sec):
            return True

        def call_async(self, request):
            requests.append(request)
            return FakeFuture()

    class FakeNode:
        def create_client(self, service_type, service_name):
            return FakeClient()

        def destroy_node(self):
            pass

    monkeypatch.setattr(query_mod.rclpy, "init", lambda args=None: None)
    monkeypatch.setattr(query_mod.rclpy, "shutdown", lambda: None)
    monkeypatch.setattr(query_mod.rclpy, "create_node", lambda name: FakeNode())
    monkeypatch.setattr(
        query_mod.rclpy,
        "spin_until_future_complete",
        lambda node, future, timeout_sec: None,
    )
    monkeypatch.setattr(
        query_mod.sys,
        "argv",
        [
            str(SCRIPT_PATH),
            "--query",
            "chair",
            "--query",
            "table",
            "--top-k",
            "3",
            "--top-k",
            "2",
            "--min-mesh-vertices",
            "37",
            "--stop-mapping",
        ],
    )

    assert query_mod.main() == 0
    assert [request.query for request in requests] == ["chair", "table"]
    assert [request.top_k for request in requests] == [3, 2]
    assert [request.min_mesh_vertices for request in requests] == [37, 37]
    assert [request.stop_mapping for request in requests] == [True, False]
    assert [request.append_to_visualization for request in requests] == [False, True]


def test_main_sends_mixed_text_and_image_queries_in_order(monkeypatch):
    requests = []

    class FakeFuture:
        def __init__(self):
            self._response = query_mod.QueryOpenVocabObjects.Response()
            self._response.success = True
            self._response.message = "ok"

        def done(self):
            return True

        def result(self):
            return self._response

    class FakeClient:
        def wait_for_service(self, timeout_sec):
            return True

        def call_async(self, request):
            requests.append(request)
            return FakeFuture()

    class FakeNode:
        def create_client(self, service_type, service_name):
            return FakeClient()

        def destroy_node(self):
            pass

    query_image = CompressedImage()
    query_image.format = "png"
    query_image.data = b"image-bytes"
    monkeypatch.setattr(query_mod, "load_query_image", lambda path: query_image)
    monkeypatch.setattr(query_mod.rclpy, "init", lambda args=None: None)
    monkeypatch.setattr(query_mod.rclpy, "shutdown", lambda: None)
    monkeypatch.setattr(query_mod.rclpy, "create_node", lambda name: FakeNode())
    monkeypatch.setattr(
        query_mod.rclpy,
        "spin_until_future_complete",
        lambda node, future, timeout_sec: None,
    )
    monkeypatch.setattr(
        query_mod.sys,
        "argv",
        [
            str(SCRIPT_PATH),
            "--query",
            "stairs",
            "--top-k",
            "1",
            "--query",
            "red car",
            "--top-k",
            "3",
            "--image",
            "/tmp/barrel.png",
            "--top-k",
            "1",
        ],
    )

    assert query_mod.main() == 0
    assert [request.query for request in requests] == ["stairs", "red car", ""]
    assert [request.top_k for request in requests] == [1, 3, 1]
    assert bytes(requests[0].query_image.data) == b""
    assert bytes(requests[1].query_image.data) == b""
    assert bytes(requests[2].query_image.data) == b"image-bytes"
    assert [request.append_to_visualization for request in requests] == [
        False,
        True,
        True,
    ]


def test_load_query_image_preserves_jpeg_bytes(tmp_path):
    image_bytes = b"\xff\xd8\xff\xe0test-jpeg-payload\xff\xd9"
    image_path = tmp_path / "potted-plant.jpg"
    image_path.write_bytes(image_bytes)

    message = query_mod.load_query_image(image_path)

    assert isinstance(message, CompressedImage)
    assert message.format == "jpeg"
    assert bytes(message.data) == image_bytes


def test_load_query_image_rejects_missing_file(tmp_path):
    image_path = tmp_path / "missing.jpg"

    with pytest.raises(FileNotFoundError, match="query image does not exist"):
        query_mod.load_query_image(image_path)


def test_load_query_image_rejects_empty_file(tmp_path):
    image_path = tmp_path / "empty.jpg"
    image_path.touch()

    with pytest.raises(ValueError, match="query image is empty"):
        query_mod.load_query_image(image_path)
