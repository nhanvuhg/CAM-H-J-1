from collections import Counter
from pathlib import Path
import re
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CATALOGS = (
    "unified_control_gui_core_vi.ts",
    "unified_control_gui_camera_vi.ts",
    "unified_control_gui_cartridge_vi.ts",
    "unified_control_gui_cpp_vi.ts",
)
PLACEHOLDER = re.compile(r"%(?:\d+|n)")
# A ROS name begins at a token boundary. Do not treat natural labels such as
# "Air/Ink" or technical pairs such as "CSI/VI" as ROS paths.
ROS_PATH = re.compile(r"(?<![A-Za-z0-9_])/(?:[A-Za-z0-9_]+/)*[A-Za-z0-9_]+")


def messages(catalog: Path):
    root = ET.parse(catalog).getroot()
    for context in root.findall("context"):
        context_name = context.findtext("name", default="")
        for message in context.findall("message"):
            source = message.findtext("source", default="")
            translation_element = message.find("translation")
            translation = "" if translation_element is None else "".join(
                translation_element.itertext()
            )
            yield context_name, source, translation_element, translation


def test_vietnamese_catalogs_are_complete_and_safe():
    translation_dir = PACKAGE_ROOT / "translations"
    for name in CATALOGS:
        catalog = translation_dir / name
        assert catalog.is_file(), f"missing catalog: {name}"
        entries = list(messages(catalog))
        assert entries, f"empty catalog: {name}"

        for context, source, element, translation in entries:
            identity = f"{name}:{context}:{source}"
            assert element is not None, f"missing translation element: {identity}"
            assert element.get("type") != "unfinished", f"unfinished: {identity}"
            assert translation.strip(), f"empty translation: {identity}"
            assert Counter(PLACEHOLDER.findall(source)) == Counter(
                PLACEHOLDER.findall(translation)
            ), f"placeholder mismatch: {identity}"
            for path in ROS_PATH.findall(source):
                assert path in translation, f"ROS path changed ({path}): {identity}"


def test_language_selector_keeps_only_supported_protocol_codes():
    selector = (PACKAGE_ROOT / "qml" / "LanguageSelector.qml").read_text(
        encoding="utf-8"
    )
    assert '{ code: "en", label: "English" }' in selector
    assert '{ code: "vi", label: "Tiếng Việt" }' in selector
    assert "languageController.setLanguage(modelData.code)" in selector


def test_language_controller_retranslates_without_reloading_qml():
    controller = (PACKAGE_ROOT / "src" / "language_controller.cpp").read_text(
        encoding="utf-8"
    )
    assert "engine_->retranslate()" in controller
    assert 'settings.setValue("ui/language", language_)' in controller
    assert "translator->isEmpty()" in controller
    assert "engine.load" not in controller
