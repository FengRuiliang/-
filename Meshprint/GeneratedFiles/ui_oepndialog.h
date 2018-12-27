/********************************************************************************
** Form generated from reading UI file 'oepndialog.ui'
**
** Created by: Qt User Interface Compiler version 5.6.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_OEPNDIALOG_H
#define UI_OEPNDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QVBoxLayout>

QT_BEGIN_NAMESPACE

class Ui_open_dia
{
public:
    QVBoxLayout *verticalLayout;
    QLabel *label;
    QDialogButtonBox *buttonBox;

    void setupUi(QDialog *open_dia)
    {
        if (open_dia->objectName().isEmpty())
            open_dia->setObjectName(QStringLiteral("open_dia"));
        open_dia->resize(400, 95);
        verticalLayout = new QVBoxLayout(open_dia);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        label = new QLabel(open_dia);
        label->setObjectName(QStringLiteral("label"));

        verticalLayout->addWidget(label);

        buttonBox = new QDialogButtonBox(open_dia);
        buttonBox->setObjectName(QStringLiteral("buttonBox"));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);

        verticalLayout->addWidget(buttonBox);


        retranslateUi(open_dia);
        QObject::connect(buttonBox, SIGNAL(accepted()), open_dia, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), open_dia, SLOT(reject()));

        QMetaObject::connectSlotsByName(open_dia);
    } // setupUi

    void retranslateUi(QDialog *open_dia)
    {
        open_dia->setWindowTitle(QApplication::translate("open_dia", "info", 0));
        label->setText(QApplication::translate("open_dia", "TextLabel", 0));
    } // retranslateUi

};

namespace Ui {
    class open_dia: public Ui_open_dia {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_OEPNDIALOG_H
