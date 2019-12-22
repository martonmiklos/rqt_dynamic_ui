#ifndef RQT_DYNAMIC_UI_WIDGET
#define RQT_DYNAMIC_UI_WIDGET

#include <QWidget>

namespace Ui {
class DynamicUIWidget;
}

class DynamicUIWidget : public QWidget
{
    Q_OBJECT

public:
    explicit DynamicUIWidget(QWidget *parent = nullptr);
    ~DynamicUIWidget();

public slots:
    void on_patheditUIFile_pathChanged(const QString &path);

private:
    Ui::DynamicUIWidget *ui;
};

#endif // RQT_DYNAMIC_UI_WIDGET
