import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Dialogs

import ImageProvider 1.0
import Model 1.0

Window {
    height: 480
    title: qsTr("Hello World")
    visible: true
    width: 640

    Model{
        id:model
    }
    ImageProvider{
        id: myImageProvider
    }

    Connections{
        target: myImageProviderQML
        function onUpdataImgChanged(){
            // console.log("oncallQmlRefeshImg:接收到图像")
            image.source = ""
            image.source = "image://myImage"
        }
    }




    Row {
        id: row

        height: 400
        width: 200

        Image {
            id: image

            fillMode: Image.PreserveAspectFit
            height: 400
            cache: false
            source: ""
            // source: "qrc:/res/SSD.png"
            // source: "image://myImage"
            width: 400
        }

        Column {
            id: column

            height: 100
            width: 200


            Switch {
                id: _switch1

                text: qsTr("打开摄像头1")

                onCheckedChanged: {
                    if (_switch1.checked) {
                        console.log("打开摄像头1");
                        myImageProvider.openCamera(0);
                    } else {
                        console.log("关闭摄像头1");
                    }
                }
            }



            Switch {
                id: _switch2

                text: qsTr("打开摄像头2")

                onCheckedChanged: {
                    if (_switch2.checked) {
                        console.log("打开摄像头2");
                        myImageProviderQML.openCamera(0);
                    } else {
                        console.log("关闭摄像头2");
                    }
                }

            }

            Switch {
                id: _switch3

                text: qsTr("打开线程摄像头")

                onCheckedChanged: {
                    if (_switch3.checked) {
                        threadController.startThread()
                        console.log("打开线程摄像头");
                    } else {
                        console.log("关闭线程摄像头");
                    }
                }

            }

            Row {
                id: row1

                height: 50
                width: 200

                TextField {
                    id: textField

                    placeholderText: qsTr("模型路径")
                }
                Button {
                    id: button

                    text: qsTr("选择模型")

                    onClicked: {
                        fileDialog.open();
                    }



                    FileDialog {
                        id: fileDialog

                        nameFilters: ["All Files (*.*)", "Text Files (*.txt)", "Images (*.png *.jpg *.jpeg)"]
                        title: "Select a File"

                        onAccepted: {
                            textField.text = fileDialog.currentFile.toString().replace("file:///", "");
                            console.log(myImageProviderQML.getOnnxpath());

                            // model.modelPath = fileDialog.currentFile.toString().replace("file:///", "");
                            myImageProviderQML.setOnnxpath(model.modelPath = fileDialog.currentFile.toString().replace("file:///", ""));
                            myImageProvider.setOnnxpath(model.modelPath = fileDialog.currentFile.toString().replace("file:///", ""));

                            console.log(myImageProviderQML.getOnnxpath());
                        }
                        onRejected: {
                            console.log("File selection was canceled");
                        }
                    }
                }
            }
            Row {
                id: row12

                height: 50
                width: 200

                TextField {
                    id: textField2

                    placeholderText: qsTr("类别路径")
                }
                Button {
                    id: button2

                    text: qsTr("选择类别")

                    onClicked: {
                        fileDialog2.open();
                    }



                    FileDialog {
                        id: fileDialog2

                        nameFilters: ["All Files (*.*)", "Text Files (*.txt)", "Images (*.png *.jpg *.jpeg)"]
                        title: "Select a File"

                        onAccepted: {
                            textField2.text = fileDialog2.currentFile.toString().replace("file:///", "");
                            // model.classNamePath = fileDialog2.currentFile.toString().replace("file:///", "");
                            myImageProviderQML.setLabels_txt_file (fileDialog2.currentFile.toString().replace("file:///", ""));
                            myImageProvider.setLabels_txt_file(model.modelPath = fileDialog2.currentFile.toString().replace("file:///", ""));

                            console.log(fileDialog2.currentFile.toString().replace("file:///", ""));
                        }
                        onRejected: {
                            console.log("File selection was canceled");
                        }
                    }
                }
            }
            Button{
                id: button1
                text: qsTr("开始识别")
                onClicked: {
                    model.startChanged();
                    myImageProvider.startChanged()

                }
            }
        }
    }
}
