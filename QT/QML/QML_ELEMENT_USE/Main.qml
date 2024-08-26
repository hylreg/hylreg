import QtQuick
import messaging 1.0

Window {
    width: 640
    height: 480
    visible: true
    title: qsTr("Hello World")

    Message {
        id: message
        author: "Amelie"
        creationDate: new Date()
    }

    Text {
        text: "Author: " + message.author + ", Created on: " + message.creationDate.toLocaleString()
        anchors.centerIn: parent
    }
}
