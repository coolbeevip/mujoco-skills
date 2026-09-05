import hashlib
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from assets import verify
import assets
import io


def entry():
    return [{"path": "model.xml", "sha256": hashlib.sha256(b"valid").hexdigest()}]


def test_missing_asset_has_recovery_diagnostic(tmp_path):
    with pytest.raises(ValueError, match="missing.*prepare"):
        verify(tmp_path, entry())


def test_corrupted_asset_is_not_accepted(tmp_path):
    (tmp_path / "model.xml").write_bytes(b"broken")
    with pytest.raises(ValueError, match="checksum"):
        verify(tmp_path, entry())


def test_verified_asset_can_be_reused(tmp_path):
    (tmp_path / "model.xml").write_bytes(b"valid")
    assert verify(tmp_path, entry()) == tmp_path


def test_prepare_is_explicit_and_reuses_verified_cache(tmp_path, monkeypatch):
    monkeypatch.setattr(
        assets,
        "lock",
        lambda: {"files": [{**entry()[0], "url": "https://example.invalid/model"}]},
    )
    monkeypatch.setattr(assets, "urlopen", lambda *a, **k: io.BytesIO(b"valid"))
    assert assets.prepare(tmp_path) == tmp_path

    def offline(*args, **kwargs):
        raise AssertionError("verified cache must not use network")

    monkeypatch.setattr(assets, "urlopen", offline)
    assert assets.prepare(tmp_path) == tmp_path


def test_bad_download_does_not_replace_existing_file(tmp_path, monkeypatch):
    (tmp_path / "model.xml").write_bytes(b"old")
    monkeypatch.setattr(
        assets,
        "lock",
        lambda: {"files": [{**entry()[0], "url": "https://example.invalid/model"}]},
    )
    monkeypatch.setattr(assets, "urlopen", lambda *a, **k: io.BytesIO(b"bad-download"))
    with pytest.raises(ValueError, match="download checksum"):
        assets.prepare(tmp_path)
    assert (tmp_path / "model.xml").read_bytes() == b"old"
    assert len(list(tmp_path.iterdir())) == 1
