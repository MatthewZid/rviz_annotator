#include <rviz_annotator/edit_widget.h>

namespace rviz_annotator
{

EditWidget::EditWidget(QWidget* parent)
: QComboBox(parent)
{
	timer = new QTimer;
	connect(timer, SIGNAL(timeout()), this, SLOT(updateList()));
	timer->setSingleShot(true);
}

void EditWidget::leaveEvent(QEvent* event)
{
	timer->start(3000);
}

void EditWidget::enterEvent(QEvent* event)
{
	timer->stop();
}

void EditWidget::updateList()
{
	if(this->currentText() == "")
		return;

	int item_index = this->findText(this->currentText());

	if(item_index == -1)
	{
		this->addItem(this->currentText());
		this->setCurrentIndex(this->count() - 1);
	}
	else
		this->setCurrentIndex(item_index);
}

} //end namespace