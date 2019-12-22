#include <rqt_dynamic_ui/rqt_dynamic_ui_widget.h>
#include "ui_rqt_dynamic_ui_widget.h"

#include <QFile>
#include <QUiLoader>

DynamicUIWidget::DynamicUIWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::DynamicUIWidget)
{
    ui->setupUi(this);
}

DynamicUIWidget::~DynamicUIWidget()
{
    delete ui;
}

void DynamicUIWidget::saveSettings(qt_gui_cpp::Settings &plugin_settings,
                                   qt_gui_cpp::Settings &instance_settings) const
{
    Q_UNUSED(plugin_settings)
    instance_settings.setValue("lastUIFilePath", ui->patheditUIFile->path());
}

void DynamicUIWidget::restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                      const qt_gui_cpp::Settings &instance_settings)
{
    Q_UNUSED(plugin_settings)
    QString uiPath = instance_settings.value("lastUIFilePath").toString();
    if (!uiPath.isEmpty())
        ui->patheditUIFile->setPath(uiPath);
}

void DynamicUIWidget::on_patheditUIFile_pathChanged(const QString &path)
{
    if (QFile::exists(path)) {
        QUiLoader loader;
        QFile file(path);
        if (file.open(QFile::ReadOnly)) {
            if (m_widget) {
                ui->gridLayoutLoadedUI->removeWidget(m_widget);
                m_widget->deleteLater();
            }

            m_widget = loader.load(&file, this);
            file.close();
            ui->gridLayoutLoadedUI->addWidget(m_widget);
        }
    }
}

void DynamicUIWidget::on_toolButtonReload_clicked()
{
    on_patheditUIFile_pathChanged(ui->patheditUIFile->path());
}
