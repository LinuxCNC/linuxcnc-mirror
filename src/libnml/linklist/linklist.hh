/********************************************************************
* Description: linklist.hh
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

#ifndef LINKED_LIST_HH
#define LINKED_LIST_HH

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>

#ifdef __cplusplus
}
#endif
enum LIST_SIZING_MODE {
    DELETE_FROM_HEAD,
    DELETE_FROM_TAIL,
    STOP_AT_MAX,
    NO_MAXIMUM_SIZE
};

class LinkedListNode {
  public:
    void *data;
    size_t size;
    int id;
    int copied;
    LinkedListNode *next;
    LinkedListNode *last;
    friend class LinkedList;
      LinkedListNode(void *_data, size_t _size);
     ~LinkedListNode();
};

class LinkedList {
  protected:
    LinkedListNode * head;
    LinkedListNode *tail;
    LinkedListNode *current_node;
    LinkedListNode *extra_node;
    int next_node_id;
  public:
    int get_current_id();
    int list_size;
    int max_list_size;
    LIST_SIZING_MODE sizing_mode;
    void set_list_sizing_mode(int, LIST_SIZING_MODE);
    void set_max_list_size(int);
    size_t last_size_retrieved;
    int delete_data_not_copied;
    void *last_data_retrieved;
    int last_copied_retrieved;
    void *retrieve_head();
    void *retrieve_tail();
    size_t last_size_stored;
    void *last_data_stored;
    int store_at_head(void *_data, size_t _size, int _copy);
    int store_at_tail(void *_data, size_t _size, int _copy);
    int store_after_current_node(void *_data, size_t _size, int _copy);
    int store_before_current_node(void *_data, size_t _size, int _copy);
    int get_newest_id() {
	return (next_node_id - 1);
    }
    void *get_head();
    void *get_tail();
    void *get_next();
    void *get_last();
    void *find_node(int _node_number);
    void delete_node(int _id);
    void delete_current_node();
    void *get_by_id(int _id);
    void *get_first_newer(int _id);
    void *get_last_newer(int _id);
    bool is_empty();
    void flush_list();
    void delete_members();

    LinkedList();
    ~LinkedList();

    // Don't copy me.
    LinkedList(const LinkedList & ll) = delete;
    LinkedList& operator=(const LinkedList & ll) = delete;
};

#endif /* LINKED_LIST_HH */
