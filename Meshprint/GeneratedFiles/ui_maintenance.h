/********************************************************************************
** Form generated from reading UI file 'maintenance.ui'
**
** Created by: Qt User Interface Compiler version 5.6.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINTENANCE_H
#define UI_MAINTENANCE_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDialog>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MyDialog
{
public:
    QVBoxLayout *verticalLayout_5;
    QVBoxLayout *verticalLayout;
    QTabWidget *tabWidget;
    QWidget *widget;
    QWidget *verticalLayoutWidget_4;
    QVBoxLayout *verticalLayout_4;
    QSpacerItem *verticalSpacer;
    QHBoxLayout *horizontalLayout_2;
    QCheckBox *intersect_checkBox;
    QLabel *intersect_label;
    QLabel *label_2;
    QSpacerItem *horizontalSpacer_2;
    QPushButton *pushButton_8;
    QPushButton *pushButton_15;
    QHBoxLayout *horizontalLayout_5;
    QCheckBox *overlap_checkBox;
    QLabel *overlap_label;
    QLabel *label_5;
    QSpacerItem *horizontalSpacer_5;
    QPushButton *pushButton_12;
    QHBoxLayout *horizontalLayout_9;
    QCheckBox *reverse_checkBox;
    QLabel *reverse_label;
    QLabel *label_6;
    QSpacerItem *horizontalSpacer_6;
    QPushButton *pushButton_13;
    QHBoxLayout *horizontalLayout_10;
    QCheckBox *badside_checkBox;
    QLabel *badside_label;
    QLabel *label_12;
    QSpacerItem *horizontalSpacer_7;
    QPushButton *pushButton_14;
    QHBoxLayout *horizontalLayout_4;
    QCheckBox *hole_checkBox;
    QLabel *hole_label;
    QLabel *label_4;
    QSpacerItem *horizontalSpacer_4;
    QPushButton *pushButton_18;
    QHBoxLayout *horizontalLayout_3;
    QCheckBox *shell_checkBox;
    QLabel *shell_label;
    QLabel *label_3;
    QSpacerItem *horizontalSpacer_3;
    QPushButton *pushButton_16;
    QSpacerItem *verticalSpacer_4;
    QHBoxLayout *horizontalLayout;
    QSpacerItem *horizontalSpacer;
    QPushButton *pushButton_17;
    QPushButton *pushButton;
    QPushButton *pushButton_2;
    QWidget *widget_2;
    QWidget *horizontalLayoutWidget;
    QHBoxLayout *horizontalLayout_6;
    QVBoxLayout *verticalLayout_2;
    QSpacerItem *verticalSpacer_2;
    QPushButton *pushButton_9;
    QSpacerItem *verticalSpacer_3;
    QPushButton *pushButton_6;
    QPushButton *pushButton_5;
    QVBoxLayout *verticalLayout_3;
    QSpacerItem *verticalSpacer_6;
    QHBoxLayout *horizontalLayout_7;
    QPushButton *pushButton_4;
    QSpacerItem *verticalSpacer_5;
    QHBoxLayout *horizontalLayout_8;
    QPushButton *pushButton_11;
    QPushButton *pushButton_10;

    void setupUi(QDialog *MyDialog)
    {
        if (MyDialog->objectName().isEmpty())
            MyDialog->setObjectName(QStringLiteral("MyDialog"));
        MyDialog->resize(440, 445);
        MyDialog->setMinimumSize(QSize(440, 445));
        MyDialog->setMaximumSize(QSize(440, 445));
        verticalLayout_5 = new QVBoxLayout(MyDialog);
        verticalLayout_5->setObjectName(QStringLiteral("verticalLayout_5"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        tabWidget = new QTabWidget(MyDialog);
        tabWidget->setObjectName(QStringLiteral("tabWidget"));
        widget = new QWidget();
        widget->setObjectName(QStringLiteral("widget"));
        verticalLayoutWidget_4 = new QWidget(widget);
        verticalLayoutWidget_4->setObjectName(QStringLiteral("verticalLayoutWidget_4"));
        verticalLayoutWidget_4->setGeometry(QRect(0, 0, 391, 331));
        verticalLayout_4 = new QVBoxLayout(verticalLayoutWidget_4);
        verticalLayout_4->setObjectName(QStringLiteral("verticalLayout_4"));
        verticalLayout_4->setContentsMargins(10, 10, 0, 0);
        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_4->addItem(verticalSpacer);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        horizontalLayout_2->setContentsMargins(-1, 10, -1, -1);
        intersect_checkBox = new QCheckBox(verticalLayoutWidget_4);
        intersect_checkBox->setObjectName(QStringLiteral("intersect_checkBox"));
        intersect_checkBox->setEnabled(true);
        intersect_checkBox->setMaximumSize(QSize(191, 16777215));
        intersect_checkBox->setMouseTracking(true);
        intersect_checkBox->setAcceptDrops(false);
        intersect_checkBox->setCheckable(true);
        intersect_checkBox->setChecked(true);
        intersect_checkBox->setAutoRepeat(false);
        intersect_checkBox->setTristate(false);

        horizontalLayout_2->addWidget(intersect_checkBox);

        intersect_label = new QLabel(verticalLayoutWidget_4);
        intersect_label->setObjectName(QStringLiteral("intersect_label"));

        horizontalLayout_2->addWidget(intersect_label);

        label_2 = new QLabel(verticalLayoutWidget_4);
        label_2->setObjectName(QStringLiteral("label_2"));

        horizontalLayout_2->addWidget(label_2);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer_2);

        pushButton_8 = new QPushButton(verticalLayoutWidget_4);
        pushButton_8->setObjectName(QStringLiteral("pushButton_8"));

        horizontalLayout_2->addWidget(pushButton_8);

        pushButton_15 = new QPushButton(verticalLayoutWidget_4);
        pushButton_15->setObjectName(QStringLiteral("pushButton_15"));

        horizontalLayout_2->addWidget(pushButton_15);


        verticalLayout_4->addLayout(horizontalLayout_2);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QStringLiteral("horizontalLayout_5"));
        horizontalLayout_5->setContentsMargins(-1, 10, -1, -1);
        overlap_checkBox = new QCheckBox(verticalLayoutWidget_4);
        overlap_checkBox->setObjectName(QStringLiteral("overlap_checkBox"));
        overlap_checkBox->setEnabled(true);
        overlap_checkBox->setMaximumSize(QSize(191, 16777215));
        overlap_checkBox->setChecked(true);

        horizontalLayout_5->addWidget(overlap_checkBox);

        overlap_label = new QLabel(verticalLayoutWidget_4);
        overlap_label->setObjectName(QStringLiteral("overlap_label"));

        horizontalLayout_5->addWidget(overlap_label);

        label_5 = new QLabel(verticalLayoutWidget_4);
        label_5->setObjectName(QStringLiteral("label_5"));

        horizontalLayout_5->addWidget(label_5);

        horizontalSpacer_5 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_5->addItem(horizontalSpacer_5);

        pushButton_12 = new QPushButton(verticalLayoutWidget_4);
        pushButton_12->setObjectName(QStringLiteral("pushButton_12"));

        horizontalLayout_5->addWidget(pushButton_12);


        verticalLayout_4->addLayout(horizontalLayout_5);

        horizontalLayout_9 = new QHBoxLayout();
        horizontalLayout_9->setObjectName(QStringLiteral("horizontalLayout_9"));
        horizontalLayout_9->setContentsMargins(-1, 10, -1, -1);
        reverse_checkBox = new QCheckBox(verticalLayoutWidget_4);
        reverse_checkBox->setObjectName(QStringLiteral("reverse_checkBox"));
        reverse_checkBox->setMaximumSize(QSize(191, 16777215));
        reverse_checkBox->setChecked(true);

        horizontalLayout_9->addWidget(reverse_checkBox);

        reverse_label = new QLabel(verticalLayoutWidget_4);
        reverse_label->setObjectName(QStringLiteral("reverse_label"));

        horizontalLayout_9->addWidget(reverse_label);

        label_6 = new QLabel(verticalLayoutWidget_4);
        label_6->setObjectName(QStringLiteral("label_6"));

        horizontalLayout_9->addWidget(label_6);

        horizontalSpacer_6 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_9->addItem(horizontalSpacer_6);

        pushButton_13 = new QPushButton(verticalLayoutWidget_4);
        pushButton_13->setObjectName(QStringLiteral("pushButton_13"));

        horizontalLayout_9->addWidget(pushButton_13);


        verticalLayout_4->addLayout(horizontalLayout_9);

        horizontalLayout_10 = new QHBoxLayout();
        horizontalLayout_10->setObjectName(QStringLiteral("horizontalLayout_10"));
        horizontalLayout_10->setContentsMargins(-1, 10, -1, -1);
        badside_checkBox = new QCheckBox(verticalLayoutWidget_4);
        badside_checkBox->setObjectName(QStringLiteral("badside_checkBox"));
        badside_checkBox->setMaximumSize(QSize(191, 16777215));
        badside_checkBox->setChecked(true);

        horizontalLayout_10->addWidget(badside_checkBox);

        badside_label = new QLabel(verticalLayoutWidget_4);
        badside_label->setObjectName(QStringLiteral("badside_label"));

        horizontalLayout_10->addWidget(badside_label);

        label_12 = new QLabel(verticalLayoutWidget_4);
        label_12->setObjectName(QStringLiteral("label_12"));

        horizontalLayout_10->addWidget(label_12);

        horizontalSpacer_7 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_10->addItem(horizontalSpacer_7);

        pushButton_14 = new QPushButton(verticalLayoutWidget_4);
        pushButton_14->setObjectName(QStringLiteral("pushButton_14"));

        horizontalLayout_10->addWidget(pushButton_14);


        verticalLayout_4->addLayout(horizontalLayout_10);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QStringLiteral("horizontalLayout_4"));
        horizontalLayout_4->setContentsMargins(-1, 10, -1, -1);
        hole_checkBox = new QCheckBox(verticalLayoutWidget_4);
        hole_checkBox->setObjectName(QStringLiteral("hole_checkBox"));
        hole_checkBox->setMaximumSize(QSize(191, 16777215));
        hole_checkBox->setChecked(true);

        horizontalLayout_4->addWidget(hole_checkBox);

        hole_label = new QLabel(verticalLayoutWidget_4);
        hole_label->setObjectName(QStringLiteral("hole_label"));

        horizontalLayout_4->addWidget(hole_label);

        label_4 = new QLabel(verticalLayoutWidget_4);
        label_4->setObjectName(QStringLiteral("label_4"));

        horizontalLayout_4->addWidget(label_4);

        horizontalSpacer_4 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_4->addItem(horizontalSpacer_4);

        pushButton_18 = new QPushButton(verticalLayoutWidget_4);
        pushButton_18->setObjectName(QStringLiteral("pushButton_18"));

        horizontalLayout_4->addWidget(pushButton_18);


        verticalLayout_4->addLayout(horizontalLayout_4);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        horizontalLayout_3->setContentsMargins(-1, 10, -1, -1);
        shell_checkBox = new QCheckBox(verticalLayoutWidget_4);
        shell_checkBox->setObjectName(QStringLiteral("shell_checkBox"));
        shell_checkBox->setMaximumSize(QSize(191, 16777215));
        shell_checkBox->setChecked(true);

        horizontalLayout_3->addWidget(shell_checkBox);

        shell_label = new QLabel(verticalLayoutWidget_4);
        shell_label->setObjectName(QStringLiteral("shell_label"));

        horizontalLayout_3->addWidget(shell_label);

        label_3 = new QLabel(verticalLayoutWidget_4);
        label_3->setObjectName(QStringLiteral("label_3"));

        horizontalLayout_3->addWidget(label_3);

        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_3->addItem(horizontalSpacer_3);

        pushButton_16 = new QPushButton(verticalLayoutWidget_4);
        pushButton_16->setObjectName(QStringLiteral("pushButton_16"));

        horizontalLayout_3->addWidget(pushButton_16);


        verticalLayout_4->addLayout(horizontalLayout_3);

        verticalSpacer_4 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_4->addItem(verticalSpacer_4);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        horizontalLayout->setContentsMargins(-1, 10, -1, -1);
        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer);

        pushButton_17 = new QPushButton(verticalLayoutWidget_4);
        pushButton_17->setObjectName(QStringLiteral("pushButton_17"));

        horizontalLayout->addWidget(pushButton_17);

        pushButton = new QPushButton(verticalLayoutWidget_4);
        pushButton->setObjectName(QStringLiteral("pushButton"));

        horizontalLayout->addWidget(pushButton);

        pushButton_2 = new QPushButton(verticalLayoutWidget_4);
        pushButton_2->setObjectName(QStringLiteral("pushButton_2"));
        pushButton_2->setEnabled(true);
        pushButton_2->setStyleSheet(QLatin1String("border-image: url(:/Meshprint/Resources/images/button_update_1.png);\n"
"border-image: url(:/Meshprint/Resources/images/button_update_2.png);"));
        QIcon icon;
        icon.addFile(QStringLiteral(":/Meshprint/Resources/images/button_update_1.png"), QSize(), QIcon::Normal, QIcon::Off);
        pushButton_2->setIcon(icon);

        horizontalLayout->addWidget(pushButton_2);


        verticalLayout_4->addLayout(horizontalLayout);

        tabWidget->addTab(widget, QString());
        widget_2 = new QWidget();
        widget_2->setObjectName(QStringLiteral("widget_2"));
        horizontalLayoutWidget = new QWidget(widget_2);
        horizontalLayoutWidget->setObjectName(QStringLiteral("horizontalLayoutWidget"));
        horizontalLayoutWidget->setGeometry(QRect(-1, -1, 401, 271));
        horizontalLayout_6 = new QHBoxLayout(horizontalLayoutWidget);
        horizontalLayout_6->setObjectName(QStringLiteral("horizontalLayout_6"));
        horizontalLayout_6->setContentsMargins(0, 0, 0, 0);
        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        verticalLayout_2->setContentsMargins(0, -1, -1, -1);
        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_2->addItem(verticalSpacer_2);

        pushButton_9 = new QPushButton(horizontalLayoutWidget);
        pushButton_9->setObjectName(QStringLiteral("pushButton_9"));

        verticalLayout_2->addWidget(pushButton_9);

        verticalSpacer_3 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_2->addItem(verticalSpacer_3);

        pushButton_6 = new QPushButton(horizontalLayoutWidget);
        pushButton_6->setObjectName(QStringLiteral("pushButton_6"));

        verticalLayout_2->addWidget(pushButton_6);

        pushButton_5 = new QPushButton(horizontalLayoutWidget);
        pushButton_5->setObjectName(QStringLiteral("pushButton_5"));

        verticalLayout_2->addWidget(pushButton_5);


        horizontalLayout_6->addLayout(verticalLayout_2);

        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        verticalLayout_3->setContentsMargins(0, -1, -1, -1);
        verticalSpacer_6 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_3->addItem(verticalSpacer_6);

        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setObjectName(QStringLiteral("horizontalLayout_7"));
        pushButton_4 = new QPushButton(horizontalLayoutWidget);
        pushButton_4->setObjectName(QStringLiteral("pushButton_4"));

        horizontalLayout_7->addWidget(pushButton_4);


        verticalLayout_3->addLayout(horizontalLayout_7);

        verticalSpacer_5 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_3->addItem(verticalSpacer_5);

        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setObjectName(QStringLiteral("horizontalLayout_8"));
        pushButton_11 = new QPushButton(horizontalLayoutWidget);
        pushButton_11->setObjectName(QStringLiteral("pushButton_11"));

        horizontalLayout_8->addWidget(pushButton_11);

        pushButton_10 = new QPushButton(horizontalLayoutWidget);
        pushButton_10->setObjectName(QStringLiteral("pushButton_10"));

        horizontalLayout_8->addWidget(pushButton_10);


        verticalLayout_3->addLayout(horizontalLayout_8);


        horizontalLayout_6->addLayout(verticalLayout_3);

        tabWidget->addTab(widget_2, QString());

        verticalLayout->addWidget(tabWidget);


        verticalLayout_5->addLayout(verticalLayout);


        retranslateUi(MyDialog);
        QObject::connect(pushButton_10, SIGNAL(clicked()), MyDialog, SLOT(reject()));
        QObject::connect(MyDialog, SIGNAL(rejected()), MyDialog, SLOT(DialogClose()));
        QObject::connect(pushButton_2, SIGNAL(clicked()), MyDialog, SLOT(on_updateButton_clicked()));
        QObject::connect(pushButton, SIGNAL(clicked()), MyDialog, SLOT(reject()));
        QObject::connect(pushButton_9, SIGNAL(clicked()), MyDialog, SLOT(on_maintenanceButton_clicked()));
        QObject::connect(pushButton_5, SIGNAL(clicked()), MyDialog, SLOT(on_createTriangle_clicked()));
        QObject::connect(pushButton_18, SIGNAL(clicked()), MyDialog, SLOT(on_repairhole_clicked()));
        QObject::connect(pushButton_8, SIGNAL(clicked()), MyDialog, SLOT(on_markcolorbutton_clicked()));
        QObject::connect(pushButton_15, SIGNAL(clicked()), MyDialog, SLOT(on_clearButton_clicked()));
        QObject::connect(intersect_checkBox, SIGNAL(clicked()), MyDialog, SLOT(on_intersectAndoverlapCheckBox()));
        QObject::connect(pushButton_14, SIGNAL(clicked()), MyDialog, SLOT(on_markbadisdebutton_clicked()));
        QObject::connect(pushButton_17, SIGNAL(clicked()), MyDialog, SLOT(on_maintenanceButton_clicked()));

        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(MyDialog);
    } // setupUi

    void retranslateUi(QDialog *MyDialog)
    {
        MyDialog->setWindowTitle(QApplication::translate("MyDialog", "Dialog", 0));
        intersect_checkBox->setText(QString());
        intersect_label->setText(QApplication::translate("MyDialog", "0", 0));
        label_2->setText(QApplication::translate("MyDialog", "intersecting triangles", 0));
        pushButton_8->setText(QApplication::translate("MyDialog", "Mark", 0));
        pushButton_15->setText(QApplication::translate("MyDialog", "Remove", 0));
        overlap_checkBox->setText(QString());
        overlap_label->setText(QApplication::translate("MyDialog", "0", 0));
        label_5->setText(QApplication::translate("MyDialog", "Overlap Triangles", 0));
        pushButton_12->setText(QApplication::translate("MyDialog", "Mark", 0));
        reverse_checkBox->setText(QString());
        reverse_label->setText(QApplication::translate("MyDialog", "0", 0));
        label_6->setText(QApplication::translate("MyDialog", "Revers Normal", 0));
        pushButton_13->setText(QApplication::translate("MyDialog", "Mark", 0));
        badside_checkBox->setText(QString());
        badside_label->setText(QApplication::translate("MyDialog", "0", 0));
        label_12->setText(QApplication::translate("MyDialog", "Edege error", 0));
        pushButton_14->setText(QApplication::translate("MyDialog", "Mark", 0));
        hole_checkBox->setText(QString());
        hole_label->setText(QApplication::translate("MyDialog", "0", 0));
        label_4->setText(QApplication::translate("MyDialog", "Hole", 0));
        pushButton_18->setText(QApplication::translate("MyDialog", "Mark", 0));
        shell_checkBox->setText(QString());
        shell_label->setText(QApplication::translate("MyDialog", "0", 0));
        label_3->setText(QApplication::translate("MyDialog", "Shell", 0));
        pushButton_16->setText(QApplication::translate("MyDialog", "Mark", 0));
        pushButton_17->setText(QApplication::translate("MyDialog", "Auto Repair", 0));
        pushButton->setText(QApplication::translate("MyDialog", "Close", 0));
        pushButton_2->setText(QApplication::translate("MyDialog", "Update", 0));
        tabWidget->setTabText(tabWidget->indexOf(widget), QApplication::translate("MyDialog", "Diagnose", 0));
        pushButton_9->setText(QApplication::translate("MyDialog", "auto repair", 0));
        pushButton_6->setText(QApplication::translate("MyDialog", "choose tirangle", 0));
        pushButton_5->setText(QApplication::translate("MyDialog", "create triangle", 0));
        pushButton_4->setText(QApplication::translate("MyDialog", "color", 0));
        pushButton_11->setText(QApplication::translate("MyDialog", "help", 0));
        pushButton_10->setText(QApplication::translate("MyDialog", "close", 0));
        tabWidget->setTabText(tabWidget->indexOf(widget_2), QApplication::translate("MyDialog", "Repair", 0));
    } // retranslateUi

};

namespace Ui {
    class MyDialog: public Ui_MyDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINTENANCE_H
