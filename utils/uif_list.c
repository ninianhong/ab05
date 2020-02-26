/*
*********************************************************************************************************
*
*                                           APPLICATION CODE
*                                         ATMEL ATSAMA5D3X-EK
*
* Filename      : uif_list.c
* Version       : V0.0.1
* Programmer(s) : Leo
* Editor        : 
*********************************************************************************************************
* Note(s)       : abstract data structure that develop for AB04 board object to manager it's elements;
*********************************************************************************************************
*/

#include <stdlib.h>
#include <string.h>

#include "uif_list.h"

//List portsList;
//List ssc0_data;
/*
*********************************************************************************************************
*                                               list_init()
*
* Description : initialize list
*
* Arguments   : List: list point
*               data  : callback function
* Returns     : none.
*
* Note(s)     : none
*********************************************************************************************************
*/
void list_init(List *list, void (*destroy)(void *data))
{
	list->size = 0;
	list->destroy = destroy;
	list->head = NULL;
	list->tail = NULL;

}

/*
*********************************************************************************************************
*                                               list_destroy()
*
* Description : destroy list
*
* Arguments   : List: list point
*               
* Returns     : none.
*
* Note(s)     : none
*********************************************************************************************************
*/
void list_destroy(List *list)
{
	void *data;

	//remove each one
	while (list_size(list) > 0)
	{
		if (list_rem_next(list, NULL, (void **)&data) == 0 && list->destroy != NULL)
		{
			list->destroy(data);
		}
	}
	//clear memory that list used
	memset(list, 0, sizeof(List));
	return;
}

/*
*********************************************************************************************************
*                                               list_ins_next()
*
* Description : insert element to list
*
* Arguments   : List:    list point
*             : element: the element indicate where the new element insert
*             : data:    new element will be inserted
*
* Returns     : 0/1:successful/fail.
*
* Note(s)     : none
*********************************************************************************************************
*/
int list_ins_next(List *list, ListElmt *element, const void *data)
{
	ListElmt *new_element;

	if ((new_element = (ListElmt *)malloc(sizeof(ListElmt))) == NULL)
		return -1;

	new_element->data = (void *)data;
	if (element == NULL)
	{
		if (list_size(list) == 0)
			list->tail = new_element;

		new_element->next = list->head;
		list->head = new_element;
	}
	else
	{
		if (element->next == NULL)
			list->tail = new_element;

		new_element->next = element->next;
		element->next = new_element;
	}

	list->size++;
	return 0;
}

/*
*********************************************************************************************************
*                                               list_rem_next()
*
* Description : remove element from list
*
* Arguments   : List:    list point
*             : element: the element indicate where the new element removed
*             : data:    the element will be removed
*
* Returns     : 0/1:successful/fail.
*
* Note(s)     : none
*********************************************************************************************************
*/
int list_rem_next(List *list, ListElmt *element, void **data)
{
	ListElmt *old_element;

	if (list_size(list) == 0)
		return -1;

	if (element == NULL)
	{
		*data = list->head->data;
		old_element = list->head;
		list->head = list->head->next;

		if (list_size(list) == 1)
			list->tail = NULL;
	}
	else
	{
		if (element->next == NULL)
			return -1;

		*data = element->next->data;
		old_element = element->next;
		element->next = element->next->next;

		if (element->next == NULL)
			list->tail = element;
	}
	free(old_element);
	list->size--;
	return 0;
}
