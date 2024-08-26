#include "message.h"

Message::Message(QObject *parent)
    : QObject{parent}
{}

QString Message::author() const {
    return m_author;
}

void Message::setAuthor(const QString &author) {
    if (m_author != author) {
        m_author = author;
        emit authorChanged();
    }
}

QDateTime Message::creationDate() const {
    return m_creationDate;
}

void Message::setCreationDate(const QDateTime &date) {
    if (m_creationDate != date) {
        m_creationDate = date;
        emit creationDateChanged();
    }
}
