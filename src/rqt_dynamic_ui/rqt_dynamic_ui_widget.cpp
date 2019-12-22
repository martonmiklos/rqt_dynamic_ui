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

void DynamicUIWidget::on_patheditUIFile_pathChanged(const QString &path)
{
    if (QFile::exists(path)) {
        QUiLoader loader;
        QFile file(path);
        if (file.open(QFile::ReadOnly)) {
            QWidget *myWidget = loader.load(&file, this);
            file.close();

            ui->gridLayoutLoadedUI->addWidget(myWidget);
        }
    }
}
