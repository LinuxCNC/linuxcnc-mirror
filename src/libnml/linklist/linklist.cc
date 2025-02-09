/********************************************************************
* Description: linklist.cc
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author:
* License: LGPL Version 2
* System: Linux
*    
* Copyright (c) 2004 All rights reserved.
*
* Last change: 
********************************************************************/

#ifdef __cplusplus
extern "C" {
#endif
#include <stdlib.h>		/* malloc() */
#include <string.h>		/* memcpy() */
#include <stdio.h>		/* fprintf(), stderr */
#ifdef __cplusplus
}
#endif
#include "linklist.hh"		/* class LinkedList */
LinkedListNode::LinkedListNode(void *_data, size_t _size)
  : data(_data),
    size(_size),
    id(0),
    copied(0),
    next(NULL),
    last(NULL)
{
}

LinkedListNode::~LinkedListNode()
{
}

LinkedList::LinkedList()
  : head(NULL),
    tail(NULL),
    current_node(NULL),
    next_node_id(0),
    list_size(0),
    max_list_size(0),
    sizing_mode(NO_MAXIMUM_SIZE),
    last_size_retrieved(0),
    delete_data_not_copied(0),
    last_data_retrieved(NULL),
    last_copied_retrieved(0),
    last_size_stored(0),
    last_data_stored(NULL)
{
    extra_node = new LinkedListNode(NULL, 0);
}

LinkedList::~LinkedList()
{
    flush_list();
    if (NULL != extra_node) {
	delete extra_node;
	extra_node = (LinkedListNode *) NULL;
    }
}

/*! Sets a sizing mode and the maximum number of nodes allowed on the
   list. The sizing mode determines what happens when there is an attempt 
   to add another node to the list after it has reached the
   _maximum_size. The following are the possible values for _new_mode:

   DELETE_FROM_TAIL: Remove one node from the tail of the list to make
   room for the new node.

   DELETE_FROM_HEAD: Remove one node from the head of the list to make
   room for the new node.

   STOP_AT_MAX: Return -1 if an attempt is made to add a new node when the 
   list is full.

   NO_MAXIMUM_SIZE: Allow the list to grow until all available memory is
   used up.

   @param _new_max_size Maximum size the list is allowed to grow to.

   @param _new_sizing_mode @see LIST_SIZING_MODE */
void LinkedList::set_list_sizing_mode(int _new_max_size,
				      LIST_SIZING_MODE _new_sizing_mode)
{
    max_list_size = _new_max_size;
    sizing_mode = _new_sizing_mode;
}

void LinkedList::flush_list()
{
    LinkedListNode *next_node;
    current_node = head;
    while (NULL != current_node) {
	next_node = current_node->next;
	if ((current_node->copied || delete_data_not_copied)
	    && (NULL != current_node->data)) {
	    free(current_node->data);
	}
	delete current_node;
	current_node = next_node;
    }
    if (last_copied_retrieved) {
	if (last_data_retrieved != NULL) {
	    free(last_data_retrieved);
	    last_data_retrieved = NULL;
	    last_size_retrieved = 0;
	}
    }
    head = (LinkedListNode *) NULL;
    tail = (LinkedListNode *) NULL;
    list_size = 0;
    last_data_stored = NULL;
    last_size_stored = 0;
}

void LinkedList::delete_members()
{
    int old_delete_data_not_copied = delete_data_not_copied;
    delete_data_not_copied = 1;
    flush_list();
    delete_data_not_copied = old_delete_data_not_copied;
}

void *LinkedList::retrieve_head()
{
    LinkedListNode *next_node;

    if (NULL != head) {
	if (last_copied_retrieved) {
	    if (NULL != last_data_retrieved) {
		free(last_data_retrieved);
		last_data_retrieved = NULL;
		last_size_retrieved = 0;
	    }
	}
	last_data_retrieved = head->data;
	last_size_retrieved = head->size;
	last_copied_retrieved = head->copied;
	next_node = head->next;
	delete head;
	head = next_node;
	if (NULL != head) {
	    head->last = (LinkedListNode *) NULL;
	} else {
	    tail = (LinkedListNode *) NULL;
	}
	list_size--;
	return (last_data_retrieved);
    }
    return (NULL);
}

void *LinkedList::retrieve_tail()
{
    LinkedListNode *last_node;

    if (NULL != tail) {
	if (last_copied_retrieved) {
	    if (NULL != last_data_retrieved) {
		free(last_data_retrieved);
		last_data_retrieved = NULL;
		last_size_retrieved = 0;
	    }
	}
	last_data_retrieved = tail->data;
	last_size_retrieved = tail->size;
	last_copied_retrieved = tail->copied;
	last_node = tail->last;
	delete tail;
	tail = last_node;
	if (NULL != tail) {
	    tail->next = (LinkedListNode *) NULL;
	} else {
	    head = (LinkedListNode *) NULL;
	}
	list_size--;
	return (last_data_retrieved);
    }
    return (NULL);
}

/*! Creates a new node and places it at the beginning of the list. If
   _copy is nonzero then this function will malloc _size bytes and copy
   _size bytes from the address starting at _data there and the get
   functions will return a pointer to the copy of the object. If _copy is
   zero then the _data pointer will be stored and the get functions will
   return a pointer to the original object.

   @param _data Pointer to the data to be stored.

   @param _size Byte count of the data.

   @param _copy If zero, just the _data pointer is stored, else a copy is
   made of the data.

   @return Returns a positive integer id that can be used to select this
   node later if successful or -1 if an error occurred. */
int LinkedList::store_at_head(void *_data, size_t _size, int _copy)
{
    LinkedListNode *new_head;
    LinkedListNode *old_tail = tail;

    if (list_size >= max_list_size) {
	switch (sizing_mode) {
	case DELETE_FROM_TAIL:
	    if (NULL != tail) {
		tail = tail->last;
		if (NULL != tail) {
		    tail->next = (LinkedListNode *) NULL;
		} else {
		    head = (LinkedListNode *) NULL;
		    delete old_tail;
		    list_size = 0;
		    break;
		}
		delete old_tail;
		list_size--;
	    }
	    break;

	case NO_MAXIMUM_SIZE:
	    break;

	case DELETE_FROM_HEAD:
	case STOP_AT_MAX:
	default:
	    return (-1);
	}
    }

    if (_copy) {
	last_data_stored = malloc(_size);
	memcpy(last_data_stored, _data, _size);
	last_size_stored = _size;
	new_head = new LinkedListNode(last_data_stored, _size);
    } else {
	last_data_stored = _data;
	last_size_stored = _size;
	new_head = new LinkedListNode(_data, _size);
    }
    if (NULL != new_head) {
	new_head->copied = _copy;
	new_head->id = next_node_id++;
	if (NULL == head) {
	    head = new_head;
	    if (NULL != tail) {
		return (-1);
	    }
	    tail = new_head;
	} else {
	    head->last = new_head;
	    new_head->last = (LinkedListNode *) NULL;
	    new_head->next = head;
	    head = new_head;
	}
	list_size++;
	return (head->id);
    } else {
	return (-1);
    }
}

/*! Creates a new node and places it at the end of the list. If _copy is
   nonzero then this function will malloc _size bytes and copy _size bytes 
   from the address starting at _data there and the get functions will
   return a pointer to the copy of the object. If _copy is zero then the
   _data pointer will be stored and the get functions will return a
   pointer to the original object.

   @param _data Pointer to the data to be stored.

   @param _size Byte count of the data.

   @param _copy If zero, just the _data pointer is stored, else a copy is
   made of the data.

   @return Returns a positive integer id that can be used to select this
   node later if successful or -1 if an error occurred. */
int LinkedList::store_at_tail(void *_data, size_t _size, int _copy)
{
    LinkedListNode *new_tail;
    LinkedListNode *old_head = head;

    if (list_size >= max_list_size) {
	switch (sizing_mode) {
	case DELETE_FROM_HEAD:
	    if (NULL != head) {
		head = head->next;
		if (NULL != head) {
		    head->last = (LinkedListNode *) NULL;
		} else {
		    head = (LinkedListNode *) NULL;
		    delete old_head;
		    list_size = 0;
		    break;
		}
		delete old_head;
		list_size--;
	    }
	    break;

	case NO_MAXIMUM_SIZE:
	    break;

	case DELETE_FROM_TAIL:
	case STOP_AT_MAX:
	default:
	    fprintf(stderr, "LinkedList: Invalid list_sizing_mode.\n");
	    return (-1);
	}
    }

    if (_copy) {
	last_data_stored = malloc(_size);
	memcpy(last_data_stored, _data, _size);
	last_size_stored = _size;
	new_tail = new LinkedListNode(last_data_stored, _size);
    } else {
	last_data_stored = _data;
	last_size_stored = _size;
	new_tail = new LinkedListNode(last_data_stored, _size);
    }
    if (NULL != new_tail) {
	new_tail->copied = _copy;
	new_tail->id = next_node_id++;
	if (NULL == tail) {
	    tail = new_tail;
	    if (NULL != head) {
		fprintf(stderr,
			"LinkedList: Tail is NULL but head is not.\n");
		return (-1);
	    }
	    head = new_tail;
	} else {
	    tail->next = new_tail;
	    new_tail->last = tail;
	    new_tail->next = (LinkedListNode *) NULL;
	    tail = new_tail;
	}
	list_size++;
	return (tail->id);
    } else {
	fprintf(stderr,
		"LinkedList: Couldn't create new node to store_at_tail.\n");
	return (-1);
    }
}

/* Creates a new node and places it after the current node. If _copy is
   nonzero then this function will malloc _size bytes and copy _size bytes 
   from the address starting at _data there and the get functions will
   return a pointer to the copy of the object. If _copy is zero then the
   _data pointer will be stored and the get functions will return a
   pointer to the original object.

   @param _data Pointer to the data to be stored.

   @param _size Byte count of the data.

   @param _copy If zero, just the _data pointer is stored, else a copy is
   made of the data.

   @return Returns a positive integer id that can be used to select this
   node later if successful or -1 if an error occurred. */

int LinkedList::store_after_current_node(void *_data, size_t _size,
					 int _copy)
{
    LinkedListNode *new_node;
    LinkedListNode *old_tail = tail;
    LinkedListNode *old_head = head;

    if (list_size >= max_list_size) {
	switch (sizing_mode) {
	case DELETE_FROM_TAIL:
	    if (NULL != tail) {
		tail = tail->last;
		if (NULL != tail) {
		    tail->next = (LinkedListNode *) NULL;
		} else {
		    head = (LinkedListNode *) NULL;
		    delete old_tail;
		    list_size = 0;
		    break;
		}
		delete old_tail;
		list_size--;
	    }
	    break;

	case NO_MAXIMUM_SIZE:
	    break;

	case DELETE_FROM_HEAD:
	    if (NULL != head) {
		head = head->next;
		if (NULL != head) {
		    head->last = (LinkedListNode *) NULL;
		} else {
		    head = (LinkedListNode *) NULL;
		    delete old_head;
		    list_size = 0;
		    break;
		}
		delete old_head;
		list_size--;
	    }
	    break;
	case STOP_AT_MAX:
	default:
	    fprintf(stderr, "LinkedList: Invalid list_sizing_mode.\n");
	    return (-1);
	}
    }

    if (_copy) {
	last_data_stored = malloc(_size);
	memcpy(last_data_stored, _data, _size);
	last_size_stored = _size;
	new_node = new LinkedListNode(last_data_stored, _size);
    } else {
	last_data_stored = _data;
	last_size_stored = _size;
	new_node = new LinkedListNode(last_data_stored, _size);
    }
    if (NULL != new_node) {
	new_node->copied = _copy;
	new_node->id = next_node_id++;
	if (NULL == current_node) {
	    if (tail == NULL) {
		tail = new_node;
		if (NULL != head) {
		    fprintf(stderr,
			    "LinkedList: Tail is NULL but the head is not.\n");
		    return (-1);
		}
		head = new_node;
	    }
	    current_node = tail;
	} else {
	    new_node->next = current_node->next;
	    if (current_node == extra_node) {
		new_node->last = current_node->last;
		if (NULL != current_node->last) {
		    current_node->last->next = new_node;
		} else {
		    head = new_node;
		}
	    } else {
		new_node->last = current_node;
	    }
	    current_node->next = new_node;
	    if (NULL != new_node->next) {
		new_node->next->last = new_node;
	    } else {
		tail = new_node;
	    }
	}
	list_size++;
	return (new_node->id);
    } else {
	fprintf(stderr,
		"LinkedList: Couldn't create new node to store_after_current.\n");
	return (-1);
    }
}

/* Creates a new node and places it before the current node. If _copy is
   nonzero then this function will malloc _size bytes and copy _size bytes 
   from the address starting at _data there and the get functions will
   return a pointer to the copy of the object. If _copy is zero then the
   _data pointer will be stored and the get functions will return a
   pointer to the original object.

   @param _data Pointer to the data to be stored.

   @param _size Byte count of the data.

   @param _copy If zero, just the _data pointer is stored, else a copy is
   made of the data.

   @return Returns a positive integer id that can be used to select this
   node later if successful or -1 if an error occurred. */

int LinkedList::store_before_current_node(void *_data, size_t _size,
					  int _copy)
{
    LinkedListNode *new_node;
    LinkedListNode *old_tail = tail;
    LinkedListNode *old_head = head;

    if (list_size >= max_list_size) {
	switch (sizing_mode) {
	case DELETE_FROM_TAIL:
	    if (NULL != tail) {
		tail = tail->last;
		if (NULL != tail) {
		    tail->next = (LinkedListNode *) NULL;
		} else {
		    head = (LinkedListNode *) NULL;
		    delete old_tail;
		    list_size = 0;
		    break;
		}
		delete old_tail;
		list_size--;
	    }
	    break;

	case NO_MAXIMUM_SIZE:
	    break;

	case DELETE_FROM_HEAD:
	    if (NULL != head) {
		head = head->next;
		if (NULL != head) {
		    head->last = (LinkedListNode *) NULL;
		} else {
		    head = (LinkedListNode *) NULL;
		    delete old_head;
		    list_size = 0;
		    break;
		}
		delete old_head;
		list_size--;
	    }
	    break;

	case STOP_AT_MAX:
	default:
	    fprintf(stderr, "LinkedList: Invalid list_sizing_mode.\n");
	    return (-1);
	}
    }

    if (_copy) {
	last_data_stored = malloc(_size);
	memcpy(last_data_stored, _data, _size);
	last_size_stored = _size;
	new_node = new LinkedListNode(last_data_stored, _size);
    } else {
	last_data_stored = _data;
	last_size_stored = _size;
	new_node = new LinkedListNode(last_data_stored, _size);
    }
    if (NULL != new_node) {
	new_node->copied = _copy;
	new_node->id = next_node_id++;
	if (NULL == current_node) {
	    if (tail == NULL) {
		tail = new_node;
		if (NULL != head) {
		    fprintf(stderr,
			    "LinkedList: Tail is NULL but head is not.\n");
		    return (-1);
		}
		head = new_node;
	    }
	    current_node = tail;
	} else {
	    new_node->last = current_node->last;
	    if (current_node == extra_node) {
		new_node->next = current_node->next;
		if (NULL != current_node->next) {
		    current_node->next->last = new_node;
		} else {
		    tail = new_node;
		}
	    } else {
		new_node->next = current_node;
	    }
	    current_node->last = new_node;
	    if (NULL != new_node->last) {
		new_node->last->next = new_node;
	    } else {
		head = new_node;
	    }
	}
	list_size++;
	return (new_node->id);
    } else {
	fprintf(stderr,
		"LinkedList: Couldn't create new node to store_before_current.\n");
	return (-1);
    }
}

/* Get the address of the first object on the list and set the current
   node to the beginning of the list.

   If the list is empty get_head returns null. Depending on how the object 
   was stored the address this function returns may be the address of the
   original object or of a copy. */
void *LinkedList::get_head()
{
    current_node = head;
    if (NULL != current_node) {
	return (current_node->data);
    } else {
	return (NULL);
    }
}

/* Get the address of the object at the end of the list and set the
   current node to the end of the list. If the list is empty get_tail
   returns null. Depending on how the object was stored the address this
   function returns may be the address of the original object or of a
   copy. */
void *LinkedList::get_tail()
{
    current_node = tail;
    if (NULL != current_node) {
	return (current_node->data);
    } else {
	return (NULL);
    }
}

/* Get the address of the next object on the list and move the current
   node one step closer to the tail.. If the list is empty get_tail
   returns null. Depending on how the object was stored the address this
   function returns may be the address of the original object or of a
   copy. */
void *LinkedList::get_next()
{
    if (NULL != current_node) {
	current_node = current_node->next;
    }
    if (NULL != current_node) {
	return (current_node->data);
    } else {
	return (NULL);
    }
}

/* Get the address of the previous object on the list and move the current 
   node one step closer to the head.. If the list is empty get_tail
   returns null. Depending on how the object was stored the address this
   function returns may be the address of the original object or of a
   copy. */
void *LinkedList::get_last()
{
    if (NULL != current_node) {
	current_node = current_node->last;
    }
    if (NULL != current_node) {
	return (current_node->data);
    } else {
	return (NULL);
    }
}

bool LinkedList::is_empty()
{
    if ((NULL == head) || (NULL == tail) || (list_size == 0)) {
	return true;
    } else {
	return false;
    }
}

void *LinkedList::get_by_id(int _id)
{
    LinkedListNode *temp;

    temp = head;
    while (NULL != temp) {
	if (temp->id == _id) {
	    return (temp->data);
	}
	temp = temp->next;
    }
    return (NULL);
}

void *LinkedList::get_first_newer(int _id)
{
    current_node = head;
    while (NULL != current_node) {
	if (current_node->id > _id) {
	    return (current_node->data);
	}
	current_node = current_node->next;
    }
    return (NULL);
}

void *LinkedList::get_last_newer(int _id)
{
    current_node = tail;
    while (NULL != current_node) {
	if (current_node->id > _id) {
	    return (current_node->data);
	}
	current_node = current_node->last;
    }
    return (NULL);
}

/* Delete the node with the associated id. */
void LinkedList::delete_node(int _id)
{
    LinkedListNode *temp;

    temp = head;
    while (NULL != temp) {
	if (temp->id == _id) {
	    list_size--;
	    if (temp == current_node) {
		if (NULL != extra_node) {
		    extra_node->next = current_node->next;
		    extra_node->last = current_node->last;
		    current_node = extra_node;
		}
	    }
	    if (NULL != temp->next) {
		temp->next->last = temp->last;
	    } else {
		tail = temp->last;
	    }
	    if (NULL != temp->last) {
		temp->last->next = temp->next;
	    } else {
		head = temp->next;
	    }
	    if ((temp->copied || delete_data_not_copied)
		&& (NULL != temp->data)) {
		free(temp->data);
	    }
	    delete temp;
	    break;
	}
	temp = temp->next;
    }
}

/* Remove the current node from the list and free any memory associated
   with it. Some extra pointers keep track of the node that was before and 
   after the deleted node so that the next call to get_next or get_last
   will return the same object as if the current node was not deleted. */
void LinkedList::delete_current_node()
{
    if (NULL != current_node && (current_node != extra_node)) {
	LinkedListNode *temp;
	temp = current_node;
	if (NULL != extra_node) {
	    extra_node->next = current_node->next;
	    extra_node->last = current_node->last;
	    current_node = extra_node;
	}
	if (NULL != temp->next) {
	    temp->next->last = temp->last;
	} else {
	    tail = temp->last;
	}
	if (NULL != temp->last) {
	    temp->last->next = temp->next;
	} else {
	    head = temp->next;
	}
	if ((temp->copied || delete_data_not_copied)
	    && (NULL != temp->data)) {
	    free(temp->data);
	}
	delete temp;
	list_size--;
    }
}

int LinkedList::get_current_id()
{
    if (current_node == NULL) {
	return (-1);
    }
    return (current_node->id);
}
