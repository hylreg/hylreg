

import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Controls.Imagine
import QtQuick.Layouts
import QtQuick.Dialogs


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
                        }

                        Button {
                            id: button
                            text: qsTr("选择")
                            font.pointSize: 15

                            onClicked: {
                                fileDialog.open()
                            }

                            FileDialog {
                                id: fileDialog

                                nameFilters: ["All Files (*.*)"]
                                title: "选择权重文件"

                                onAccepted: {
                                    textField.text = fileDialog.currentFile.toString().replace("file:///", "");
                                }
                                onRejected: {
                                    console.log("File selection was canceled");
                                }
                            }

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
                        }

                        Button {
                            id: button1
                            text: qsTr("选择")
                            font.pointSize: 15

                            onClicked: {
                                fileDialog1.open();
                            }

                            FileDialog {
                                id: fileDialog1

                                nameFilters: ["All Files (*.*)"]
                                title: "选择类别文件"

                                onAccepted: {
                                    textField1.text = fileDialog.currentFile.toString().replace("file:///", "");
                                }
                                onRejected: {
                                    console.log("File selection was canceled");
                                }
                            }
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

                    property int selectedIndex: -1

                    CheckBox {
                        id: checkBox
                        text: qsTr("OnenCV DNN")
                        Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                        Layout.fillWidth: true
                        font.pointSize: root.fontSize15

                        checked: rowLayout3.selectedIndex === 0
                        onCheckedChanged: {
                            if (checked) {
                                rowLayout3.selectedIndex = 0
                            }
                        }
                    }

                    CheckBox {
                        id: checkBox1
                        text: qsTr("OpenVINO")
                        Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                        Layout.fillWidth: true
                        font.pointSize: root.fontSize15

                        checked: rowLayout3.selectedIndex === 1
                        onCheckedChanged: {
                            if (checked) {

                                rowLayout3.selectedIndex = 1
                            }
                        }
                    }

                    CheckBox {
                        id: checkBox2
                        text: qsTr("ONNXRUNTIME")
                        Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                        Layout.fillWidth: true
                        font.pointSize: root.fontSize15

                        checked: rowLayout3.selectedIndex === 2
                        onCheckedChanged: {
                            if (checked) {
                                rowLayout3.selectedIndex = 2
                            }
                        }
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
                            height: 40
                            font.pointSize: root.fontSize15

                            from: 0
                            value: decimalToInt(0.6)
                            to: decimalToInt(1)
                            stepSize: decimalFactor/decimalFactor
                            editable: true

                            property real decimals: 2
                            property real realValue: value / decimalFactor
                            readonly property int decimalFactor: Math.pow(10, decimals)

                            function decimalToInt(decimal) {
                                return decimal * decimalFactor
                            }

                            validator: DoubleValidator {
                                bottom: Math.min(spinBox.from, spinBox.to)
                                top:  Math.max(spinBox.from, spinBox.to)
                                decimals: spinBox.decimals
                                notation: DoubleValidator.StandardNotation
                            }

                            textFromValue: function(value, locale) {
                                return Number(value / decimalFactor).toLocaleString(locale, 'f', spinBox.decimals)
                            }

                            valueFromText: function(text, locale) {
                                return Math.round(Number.fromLocaleString(locale, text) * decimalFactor)
                            }

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
                            height: 40
                            font.pointSize: root.fontSize15


                            from: 0
                            value: decimalToInt(0.6)
                            to: decimalToInt(1)
                            stepSize: decimalFactor/decimalFactor
                            editable: true

                            property real decimals: 2
                            property real realValue: value / decimalFactor
                            readonly property int decimalFactor: Math.pow(10, decimals)

                            function decimalToInt(decimal) {
                                return decimal * decimalFactor
                            }

                            validator: DoubleValidator {
                                bottom: Math.min(spinBox.from, spinBox.to)
                                top:  Math.max(spinBox.from, spinBox.to)
                                decimals: spinBox.decimals
                                notation: DoubleValidator.StandardNotation
                            }

                            textFromValue: function(value, locale) {
                                return Number(value / decimalFactor).toLocaleString(locale, 'f', spinBox.decimals)
                            }

                            valueFromText: function(text, locale) {
                                return Math.round(Number.fromLocaleString(locale, text) * decimalFactor)
                            }
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
                            height: 40
                            font.pointSize: root.fontSize15


                            from: 0
                            value: decimalToInt(0.6)
                            to: decimalToInt(1)
                            stepSize: decimalFactor/decimalFactor
                            editable: true

                            property real decimals: 2
                            property real realValue: value / decimalFactor
                            readonly property int decimalFactor: Math.pow(10, decimals)

                            function decimalToInt(decimal) {
                                return decimal * decimalFactor
                            }

                            validator: DoubleValidator {
                                bottom: Math.min(spinBox.from, spinBox.to)
                                top:  Math.max(spinBox.from, spinBox.to)
                                decimals: spinBox.decimals
                                notation: DoubleValidator.StandardNotation
                            }

                            textFromValue: function(value, locale) {
                                return Number(value / decimalFactor).toLocaleString(locale, 'f', spinBox.decimals)
                            }

                            valueFromText: function(text, locale) {
                                return Math.round(Number.fromLocaleString(locale, text) * decimalFactor)
                            }
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

                        onCheckStateChanged: {
                            if(checkState){
                                console.log("确认选中")
                            }else{
                                 console.log("取消选中")
                            }
                        }
                    }

                    CheckBox {
                        id: checkBox4
                        text: qsTr("显示FPS")
                        Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                        Layout.fillWidth: true
                        font.pointSize:root.fontSize15

                        onCheckStateChanged: {
                            if(checkState){
                                console.log("确认选中")
                            }else{
                                 console.log("取消选中")
                            }
                        }
                    }

                    CheckBox {
                        id: checkBox5
                        text: qsTr("显示类别")
                        Layout.fillWidth: true
                        Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                        font.pointSize: root.fontSize15

                        onCheckStateChanged: {
                            if(checkState){
                                console.log("确认选中")
                            }else{
                                 console.log("取消选中")
                            }
                        }
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
                title: qsTr("选择Label")

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
                        }

                        Button {
                            id: button2
                            height: 40
                            text: qsTr("选择")
                            anchors.verticalCenter: parent.verticalCenter
                            font.pointSize: 15

                            onClicked: {
                                fileDialog2.open()
                            }

                            FileDialog {
                                id: fileDialog2

                                nameFilters: ["All Files (*.*)"]
                                title: "选择权重文件"

                                onAccepted: {
                                    textField2.text = fileDialog2.currentFile.toString().replace("file:///", "");
                                }
                                onRejected: {
                                    console.log("File selection was canceled");
                                }
                            }
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

                    onClicked: {
                    }
                }

                Button {
                    id: toolButton1
                    text: qsTr("推理运行")
                    Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                    font.pointSize: 15
                    onClicked: {
                    }
                }

                Button {
                    id: toolButton2
                    text: qsTr("结束推理")
                    Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                    font.pointSize: 15
                    onClicked: {
                    }
                }

            }

            Switch {
                id: _switch
                text: qsTr("打开摄像头")
                anchors.top: rowLayout.bottom
                anchors.topMargin: 0
                anchors.horizontalCenter: parent.horizontalCenter

                onCheckedChanged: {
                    if(checked){
                        threadManager.startCamera();
                    }else{
                        threadManager.stopCamera()
                    }
                }
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
                id: videoFeed
                source: "image://camera/"
                fillMode: Image.PreserveAspectFit
                Timer {
                    interval: 30
                    running: true
                    repeat: true
                    onTriggered: videoFeed.source = "image://camera/" + Math.random()
                }
            }
        }
    }

}
