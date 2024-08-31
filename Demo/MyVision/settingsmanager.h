#ifndef SETTINGS_MANAGER_H
#define SETTINGS_MANAGER_H

#include <QObject>
#include <QSettings>

class SettingsManager : public QObject {
    Q_OBJECT
public:
    explicit SettingsManager(const QString &path, QObject *parent = nullptr);

    Q_INVOKABLE void setValue(const QString &key, const QVariant &value);
    Q_INVOKABLE QVariant value(const QString &key, const QVariant &defaultValue = QVariant());

private:
    QSettings m_settings;
};

#endif // SETTINGS_MANAGER_H
