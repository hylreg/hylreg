

import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Controls.Material
import QtQuick.Layouts

Rectangle {
    id: root
    width: 1400
    height: 900

    // Material.theme: Material.Light // 使用Material主题
    // Material.accent: Material.Blue // 设置主题的强调色（影响选中项颜色）

    property int fontSize10 : 10
    property int fontSize15 : 15


    Row {
        id: row
        anchors.fill: parent
        layoutDirection: Qt.RightToLeft

        Column {
            id: column
            width: 600
            anchors.top: parent.top
            anchors.bottom: parent.bottom
            anchors.topMargin: 0
            anchors.bottomMargin: 0

            TabBar {
                id: tabBar
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.leftMargin: 0
                anchors.rightMargin: 0




                TabButton {
                    id: tabButton
                    text: qsTr("YOLO8")
                }
                TabButton {
                    id: tabButton1
                    text: qsTr("YOLOX")
                }
                TabButton {
                    id: tabButton2
                    text: qsTr("Fast-RCNN")
                }

                TabButton {
                    id: tabButton3
                    text: qsTr("Mask-RCNN")
                }
            }

            GroupBox {
                id: groupBox
                height: 200
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.top: tabBar.bottom
                anchors.leftMargin: 0
                anchors.rightMargin: 0
                anchors.topMargin: 20
                font.pointSize: 20
                title: qsTr("模型设置")

                ColumnLayout {
                    id: columnLayout
                    anchors.fill: parent

                    RowLayout {
                        id: rowLayout1
                        Layout.fillWidth: true

                        Label {
                            id: label
                            text: qsTr("权重文件：")
                            anchors.verticalCenter: textField.verticalCenter
                            font.pointSize: 15
                        }

                        TextField {
                            id: textField
                            width: 500
                            Layout.fillWidth: true
                            placeholderText: qsTr("Text Field")
                        }

                        Button {
                            id: button
                            text: qsTr("选择")
                            font.pointSize: 15
                        }
                    }

                    RowLayout {
                        id: rowLayout2
                        Layout.fillHeight: true

                        Label {
                            id: label1
                            text: qsTr("类别文件：")
                            anchors.verticalCenter: textField1.verticalCenter
                            font.pointSize: 15
                        }

                        TextField {
                            id: textField1
                            Layout.fillWidth: true
                            placeholderText: qsTr("Text Field")
                        }

                        Button {
                            id: button1
                            text: qsTr("选择")
                            font.pointSize: 15
                        }
                    }
                }
            }

            GroupBox {
                id: groupBox1
                height: 120
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.top: groupBox.bottom
                anchors.leftMargin: 0
                anchors.rightMargin: 0
                anchors.topMargin: 0
                font.pointSize: 20
                title: qsTr("部署平台")


                RowLayout {
                    id: rowLayout3
                    anchors.verticalCenter: parent.verticalCenter
                    anchors.left: parent.left
                    anchors.right: parent.right
                    anchors.top: parent.top
                    anchors.leftMargin: 0
                    anchors.rightMargin: 0
                    anchors.topMargin: 0

                    CheckBox {
                        id: checkBox
                        text: qsTr("OnenCV DNN")
                        Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                        Layout.fillWidth: true
                        font.pointSize: root.fontSize15
                    }

                    CheckBox {
                        id: checkBox1
                        text: qsTr("OpenVINO")
                        Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                        Layout.fillWidth: true
                        font.pointSize: root.fontSize15
                    }

                    CheckBox {
                        id: checkBox2
                        text: qsTr("ONNXRUNTIME")
                        Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                        Layout.fillWidth: true
                        font.pointSize: root.fontSize15
                    }
                }
            }

            GroupBox {
                id: groupBox2
                height: 120
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.top: groupBox1.bottom
                anchors.leftMargin: 0
                anchors.rightMargin: 0
                anchors.topMargin: 0
                font.pointSize: 20
                title: qsTr("参数设置")

                RowLayout {
                    id: rowLayout4
                    anchors.fill: parent

                    Row {
                        id: row4
                        spacing: 0

                        Label {
                            id: label2
                            height: 40
                            text: qsTr("Conf：")
                            anchors.verticalCenter: spinBox.verticalCenter
                            horizontalAlignment: Text.AlignLeft
                            verticalAlignment: Text.AlignVCenter
                            font.pointSize:root.fontSize10
                        }

                        SpinBox {
                            id: spinBox
                            font.pointSize: 15
                            focus: false



                        }



                    }

                    Row {
                        id: row1
                        width: 200
                        height: 400
                        Layout.fillHeight: false
                        Layout.fillWidth: false

                        Label {
                            id: label3
                            height: 40
                            text: qsTr("Socre：")
                            anchors.verticalCenter: spinBox1.verticalCenter
                            verticalAlignment: Text.AlignVCenter
                            font.pointSize: root.fontSize10
                        }

                        SpinBox {
                            id: spinBox1
                        }
                    }

                    Row {
                        id: row2
                        width: 200
                        height: 400
                        Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                        Layout.fillHeight: false
                        Layout.fillWidth: false

                        Label {
                            id: label4
                            height: 40
                            text: qsTr("NMS：")
                            anchors.verticalCenter: spinBox2.verticalCenter
                            verticalAlignment: Text.AlignVCenter
                            font.pointSize: root.fontSize10
                        }

                        SpinBox {
                            id: spinBox2
                        }
                    }
                }
            }

            GroupBox {
                id: groupBox3
                height: 120
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.top: groupBox2.bottom
                anchors.leftMargin: 0
                anchors.rightMargin: 0
                anchors.topMargin: 0
                font.pointSize: 20
                title: qsTr("显示设置")

                RowLayout {
                    id: rowLayout5
                    anchors.fill: parent

                    CheckBox {
                        id: checkBox3
                        text: qsTr("显示Box")
                        Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                        Layout.fillWidth: true
                        font.pointSize:root.fontSize15
                    }

                    CheckBox {
                        id: checkBox4
                        text: qsTr("显示FPS")
                        Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                        Layout.fillWidth: true
                        font.pointSize:root.fontSize15
                    }

                    CheckBox {
                        id: checkBox5
                        text: qsTr("显示类别")
                        Layout.fillWidth: true
                        Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                        font.pointSize: root.fontSize15
                    }
                }
            }

            GroupBox {
                id: groupBox4
                height: 120
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.top: groupBox3.bottom
                anchors.topMargin: 0
                font.pointSize: 20
                title: qsTr("文件选择")

                ColumnLayout {
                    id: columnLayout1
                    anchors.fill: parent

                    RowLayout {
                        id: rowLayout6
                        Layout.fillWidth: true

                        Label {
                            id: label5
                            height: 40
                            text: qsTr("Label：")
                            anchors.verticalCenter: parent.verticalCenter
                            horizontalAlignment: Text.AlignLeft
                            verticalAlignment: Text.AlignVCenter
                            font.pointSize: 15
                        }

                        TextField {
                            id: textField2
                            height: 40
                            anchors.verticalCenter: parent.verticalCenter
                            Layout.fillWidth: true
                            placeholderText: qsTr("Text Field")
                        }

                        Button {
                            id: button2
                            height: 40
                            text: qsTr("选择")
                            anchors.verticalCenter: parent.verticalCenter
                            font.pointSize: 15
                        }
                    }
                }
            }

            RowLayout {
                id: rowLayout
                height: 100
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.top: groupBox4.bottom
                anchors.topMargin: 0

                Button {
                    id: toolButton
                    text: qsTr("保存/应用")
                    Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                    font.pointSize: 15
                }

                Button {
                    id: toolButton1
                    text: qsTr("推理运行")
                    Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                    font.pointSize: 15
                }

                Button {
                    id: toolButton2
                    text: qsTr("结束推理")
                    Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                    font.pointSize: 15
                }

            }

            Switch {
                id: _switch
                text: qsTr("打开摄像头")
                anchors.top: rowLayout.bottom
                anchors.topMargin: 0
                anchors.horizontalCenter: parent.horizontalCenter
            }


        }

        Frame {
            id: frame
            anchors.left: column.right
            anchors.right: parent.right
            anchors.top: parent.top
            anchors.bottom: parent.bottom
            anchors.leftMargin: 30
            anchors.rightMargin: 30
            anchors.topMargin: 30
            anchors.bottomMargin: 30

            Image {
                id: image
                anchors.fill: parent
                source: "qrc:/res/SSD.png"
                fillMode: Image.PreserveAspectFit
            }
        }
    }

}
