

import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Controls.Imagine
import QtQuick.Layouts
import QtQuick.Dialogs
import MyApp 1.0


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
                height: 150
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
                            Layout.fillHeight: true
                            Layout.fillWidth: true
                            font.pointSize: fontSize10

                            Component.onCompleted: {
                                textField.text = MyApp.settingsManager.value("ModelPath","defaultValue")
                                MyApp.modelManger.setModelPath(textField.text)
                            }

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
                                    MyApp.settingsManager.setValue("ModelPath", textField.text)
                                    MyApp.modelManger.setModelPath(textField.text)
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
                            Layout.fillHeight: true
                            Layout.fillWidth: true
                            font.pointSize: fontSize10

                            Component.onCompleted: {
                                textField1.text = MyApp.settingsManager.value("ClassPath","defaultValue")
                                MyApp.modelManger.setClassPath(textField1.text)
                            }

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
                                    textField1.text = fileDialog1.currentFile.toString().replace("file:///", "");
                                    MyApp.settingsManager.setValue("ClassPath", textField1.text)
                                    MyApp.modelManger.setClassPath(textField1.text)
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
                                MyApp.modelManger.setDeployPlatform("OnenCV DNN")
                                MyApp.settingsManager.setValue("DeployPlatform", rowLayout3.selectedIndex)

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
                                MyApp.modelManger.setDeployPlatform("OpenVINO")
                                MyApp.settingsManager.setValue("DeployPlatform", rowLayout3.selectedIndex)

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
                                MyApp.modelManger.setDeployPlatform("ONNXRUNTIME")
                                MyApp.settingsManager.setValue("DeployPlatform", rowLayout3.selectedIndex)
                            }
                        }
                    }

                    Component.onCompleted: {
                        var deployPlatformValue = parseInt(MyApp.settingsManager.value("DeployPlatform", "defaultValue"), 10);

                        if(deployPlatformValue === 0){
                            console.log(0)
                            rowLayout3.selectedIndex = 0
                        }
                        else if(deployPlatformValue===1){
                             console.log(1)
                            rowLayout3.selectedIndex = 1
                        }
                        else if(deployPlatformValue===2){
                             console.log(2)
                            rowLayout3.selectedIndex = 2
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
                        Layout.fillWidth: true
                        Layout.fillHeight: true
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
                            value: decimalToInt(parseInt(MyApp.settingsManager.value("Conf", "60"), 10)/100.0)
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

                            onValueChanged: {
                                MyApp.settingsManager.setValue("Conf", spinBox.value)

                            }

                            Component.onCompleted: {
                                var deployPlatformValue = parseInt(MyApp.settingsManager.value("Conf", "60"), 10);
                                spinBox.value = decimalToInt(deployPlatformValue/100.0)
                            }


                        }



                    }

                    Row {
                        id: row1
                        width: 200
                        height: 400
                        Layout.fillHeight: true
                        Layout.fillWidth: true

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
                            value: decimalToInt(parseInt(MyApp.settingsManager.value("Socre", "60"), 10)/100.0)
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

                            onValueChanged: {
                                MyApp.settingsManager.setValue("Socre", spinBox1.value)

                            }

                            Component.onCompleted: {
                                var deployPlatformValue = parseInt(MyApp.settingsManager.value("Socre", "60"), 10);
                                spinBox1.value = decimalToInt(deployPlatformValue/100.0)
                            }
                        }
                    }

                    Row {
                        id: row2
                        width: 200
                        height: 400
                        Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                        Layout.fillHeight: true
                        Layout.fillWidth: true

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
                            value: decimalToInt(parseInt(MyApp.settingsManager.value("NMS", "60"), 10)/100.0)
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

                            onValueChanged: {
                                MyApp.settingsManager.setValue("NMS", spinBox2.value)

                            }

                            Component.onCompleted: {
                                var deployPlatformValue = parseInt(MyApp.settingsManager.value("NMS", "60"), 10);
                                spinBox2.value = decimalToInt(deployPlatformValue/100.0)
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

                    function updateState() {
                        let state = 0;
                        if (checkBox3.checked) state |= 1;  // 0b001
                        if (checkBox4.checked) state |= 2;  // 0b010
                        if (checkBox5.checked) state |= 4;  // 0b100
                        MyApp.modelManger.setCheckBoxState(state)
                        MyApp.settingsManager.setValue("DispalyState", state)
                    }

                    CheckBox {
                        id: checkBox3
                        text: qsTr("显示Box")
                        Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                        Layout.fillWidth: true
                        font.pointSize:root.fontSize15

                        onCheckStateChanged: {
                            rowLayout5.updateState()
                        }
                    }

                    CheckBox {
                        id: checkBox4
                        text: qsTr("显示FPS")
                        Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                        Layout.fillWidth: true
                        font.pointSize:root.fontSize15

                        onCheckStateChanged: {
                            rowLayout5.updateState()
                        }
                    }

                    CheckBox {
                        id: checkBox5
                        text: qsTr("显示类别")
                        Layout.fillWidth: true
                        Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                        font.pointSize: root.fontSize15

                        onCheckStateChanged: {
                            rowLayout5.updateState()
                        }
                    }
                    Component.onCompleted: {
                        // 从设置中读取 state
                        let state = parseInt(MyApp.settingsManager.value("DispalyState", "0"), 10);

                        // 根据 state 更新复选框的状态
                        checkBox3.checked = (state & 1) !== 0;  // 0b001
                        checkBox4.checked = (state & 2) !== 0;  // 0b010
                        checkBox5.checked = (state & 4) !== 0;  // 0b100
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
                            Layout.fillHeight: true
                            Layout.fillWidth: true
                            font.pointSize: fontSize10

                            Component.onCompleted: {
                                textField2.text = MyApp.settingsManager.value("LabelPath","defaultValue").toString()
                                MyApp.modelManger.setModelPath(textField2.text)
                            }
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
                                title: "选择Label文件"

                                onAccepted: {
                                    textField2.text = fileDialog2.currentFile.toString().replace("file:///", "");
                                    MyApp.settingsManager.setValue("LabelPath", textField2.text)
                                    MyApp.modelManger.setModelPath(textField2.text)

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
                        _switch.checked = true
                        MyApp.threadManager.startCamera();
                    }
                }

                Button {
                    id: toolButton2
                    text: qsTr("结束推理")
                    Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                    font.pointSize: 15
                    onClicked: {
                        _switch.checked = false
                        MyApp.threadManager.stopCamera()
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
                        MyApp.threadManager.startCamera();
                    }else{
                        MyApp.threadManager.stopCamera()
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
                anchors.fill: parent
                source: "image://camera/"
                fillMode: Image.PreserveAspectFit
                Timer {
                    interval: 30
                    running: _switch.checked
                    repeat: true
                    onTriggered: videoFeed.source = "image://camera/" + Math.random()
                }
            }
        }
    }

}
