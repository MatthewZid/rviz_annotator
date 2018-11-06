#ifndef EDIT_WIDGET_H
#define EDIT_WIDGET_H

#include <QWidget>
#include <QComboBox>
#include <QTimer>

namespace rviz_annotator
{

class EditWidget: public QComboBox
{
Q_OBJECT
public:
	EditWidget(QWidget* parent = 0);

	virtual void leaveEvent(QEvent* event);
	virtual void enterEvent(QEvent* event);

private Q_SLOTS:
	void updateList();

private:
	QTimer* timer;
};

} //end namespace

#endif