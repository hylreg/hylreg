#include "SettingsManager.h"

SettingsManager::SettingsManager(const QString &path, QObject *parent)
    : QObject(parent), m_settings(path, QSettings::IniFormat) {
    // 可以选择 QSettings 的其他格式
}

void SettingsManager::setValue(const QString &key, const QVariant &value) {
    m_settings.setValue(key, value);
}

QVariant SettingsManager::value(const QString &key, const QVariant &defaultValue) {
    return m_settings.value(key, defaultValue);
}
