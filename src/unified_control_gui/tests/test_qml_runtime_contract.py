from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def read(relative_path: str) -> str:
    return (PACKAGE_ROOT / relative_path).read_text(encoding="utf-8")


def test_system_alert_delegate_width_does_not_depend_on_content_height():
    qml = read("qml/SystemAlertButton.qml")

    assert "alertList.width - alertScrollBar.width - 4" in qml
    assert "width: alertList.width - (alertList.contentHeight" not in qml


def test_login_popup_height_does_not_depend_on_its_content_item():
    qml = read("qml/Main.qml")

    assert "height: loginColumn.implicitHeight" not in qml
    assert "id: loginPopup" in qml
    assert "height: 600" in qml


def test_resume_popup_uses_fill_signal_warning_and_alert_icon():
    qml = read("qml/Main.qml")

    assert 'source: "qrc:/qml/icons/triangle_alert.svg"' in qml
    assert 'text: qsTr("FILL SIGNAL NOT RECEIVED")' in qml


def test_production_tab_icons_are_available_under_qml_prefix():
    root = ET.parse(PACKAGE_ROOT / "qml.qrc").getroot()
    aliases = {
        element.get("alias")
        for resource in root.findall("qresource")
        if resource.get("prefix") == "/qml"
        for element in resource.findall("file")
    }

    assert "icons/schedule.svg" in aliases
    assert "icons/droplet.svg" in aliases
