from robotnav.scene3d import write_viewer
from robotnav.terrain import solvable_map


def _page(tmp_path, **kw):
    t = solvable_map("grassland", seed=3, size=48)
    return write_viewer([t], tmp_path / "docs" / "index.html", scenes=[{"key": "grassland"}], **kw).read_text()


def test_viewer_is_a_full_standards_mode_document(tmp_path):
    page = _page(tmp_path)
    assert page.startswith("<!doctype html>\n")
    assert '<meta charset="utf-8">' in page[:200]               # the page has symbols like × and ⇄
    assert '"key":"grassland"' in page                          # the scene data went in
    assert "og:image" not in page                               # no link-preview tags unless it has a home


def test_hosted_viewer_has_link_previews(tmp_path):
    page = _page(tmp_path, site_url="https://example.github.io/demo/")
    head = page[:2500]
    assert '<meta property="og:image" content="https://example.github.io/demo/social.png">' in head
    assert '<meta name="twitter:card" content="summary_large_image">' in head
    assert '<link rel="canonical" href="https://example.github.io/demo/">' in head
