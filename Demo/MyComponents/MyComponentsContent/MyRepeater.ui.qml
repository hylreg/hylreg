

/*
This is a UI file (.ui.qml) that is intended to be edited in Qt Design Studio only.
It is supposed to be strictly declarative and only uses a subset of QML. If you edit
this file manually, you might introduce QML code that is not supported by Qt Design Studio.
Check out https://doc.qt.io/qtcreator/creator-quick-ui-forms.html for details on .ui.qml files.
*/
import QtQuick 2.15
import QtQuick.Controls 2.15

Rectangle {
    id: root
    width: 1024
    height: 768

    Column {
        id: column
        anchors.fill: parent

        Row {
            id: row
            Repeater {
                id: repeater

                model: 3
                Rectangle {
                    width: 100
                    height: 40
                    border.width: 1
                    color: "yellow"
                }
            }
        }

        Row {
            Rectangle {
                width: 10
                height: 20
                color: "red"
            }
            Repeater {
                model: 10
                Rectangle {
                    width: 20
                    height: 20
                    radius: 10
                    color: "green"
                }
            }
            Rectangle {
                width: 10
                height: 20
                color: "blue"
            }
        }

        Item {
            // Can't repeat QtObject as it doesn't derive from Item.
            Repeater {
                model: 10
                QtObject {}
            }
        }

        Column {
            Repeater {
                model: 10
                Text {
                    required property int index
                    text: "I'm item " + index
                }
            }
        }

        Column {
            Repeater {
                model: 10
                Text {
                    required property int index
                    text: "I'm item " + index
                }
            }
        }
    }
}
