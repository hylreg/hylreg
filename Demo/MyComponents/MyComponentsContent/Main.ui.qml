

/*
This is a UI file (.ui.qml) that is intended to be edited in Qt Design Studio only.
It is supposed to be strictly declarative and only uses a subset of QML. If you edit
this file manually, you might introduce QML code that is not supported by Qt Design Studio.
Check out https://doc.qt.io/qtcreator/creator-quick-ui-forms.html for details on .ui.qml files.
*/
import QtQuick 6.5
import QtQuick.Controls 6.5
import MyComponents

Rectangle {
    id: rectangle
    width: Constants.width
    height: Constants.height

    color: Constants.backgroundColor

    Row {
        id: row
        anchors.fill: parent

        Column {
            id: column
            width: 700
            height: 700

            ToolBar {
                id: toolBar
                width: 500

                Row {
                    id: row1
                    anchors.fill: parent

                    ToolButton {
                        id: toolButton
                        text: qsTr("Tool Button")
                    }

                    ToolButton {
                        id: toolButton1
                        text: qsTr("Tool Button")
                    }

                    ToolButton {
                        id: toolButton2
                        text: qsTr("Tool Button")
                    }
                }
            }

            TabBar {
                id: tabBar
                width: 500
                height: 40
                anchors.top: toolBar.bottom
                anchors.topMargin: 0

                TabButton {
                    id: tabButton
                    text: qsTr("Tab Button")

                    Connections {
                        target: tabButton
                        onClicked: {
                            tabOne.visible = true
                        }
                    }

                    TabOne {
                        id: tabOne
                        visible: false
                    }
                }

                TabButton {
                    id: tabButton1
                    text: qsTr("Tab Button")

                    Connections {
                        target: tabButton1
                        onClicked: {
                            pageLoader.source = "TabTow.ui.qml"
                        }
                    }

                    Loader {
                        id: pageLoader
                    }
                }

                TabButton {
                    id: tabButton2
                    text: qsTr("Tab Button")
                }
            }

            Image {
                id: image
                width: 600
                height: 600
                anchors.top: tabBar.bottom
                anchors.topMargin: 0
                source: "qrc:/qtquickplugin/images/template_image.png"
                fillMode: Image.PreserveAspectFit
            }
        }

        Column {
            id: column1
            anchors.left: column.right
            anchors.right: parent.right
            anchors.top: parent.top
            anchors.bottom: column.bottom
            anchors.leftMargin: 0
            anchors.rightMargin: 0
            anchors.topMargin: 0
            anchors.bottomMargin: 0

            Switch {
                id: switch1
                anchors.horizontalCenter: parent.horizontalCenter
            }

            Row {
                id: row2
                height: 40
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.top: switch1.bottom
                anchors.leftMargin: 0
                anchors.rightMargin: 0
                anchors.topMargin: 0

                TextField {
                    id: textField
                    placeholderText: qsTr("Text Field")
                }

                Button {
                    id: button
                    text: qsTr("Button")
                }
            }

            Row {
                id: row3
                height: 40
                anchors.left: row2.left
                anchors.right: parent.right
                anchors.top: row2.bottom
                anchors.leftMargin: 0
                anchors.rightMargin: 0
                anchors.topMargin: 0

                TextField {
                    id: textField1
                    placeholderText: qsTr("Text Field")
                }

                Button {
                    id: button1
                    text: qsTr("Button")
                }
            }

            Button {
                id: button2
                text: qsTr("Button")
                anchors.top: row3.bottom
                anchors.topMargin: 0
                anchors.horizontalCenter: parent.horizontalCenter
            }
        }
    }

    states: [
        State {
            name: "clicked"
        }
    ]
}
