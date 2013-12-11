#include <assert.h>

#include "k9_cfg.h"

typedef unsigned char  uint8;
typedef unsigned short uint16;
typedef unsigned int   uint32;
typedef signed char    int8;
typedef signed short   int16;
typedef signed int     int32;

struct list {
  struct list *prev, *next;
};

#define LIST_INIT(li)   ((li)->prev = (li)->next = (li))
#define LIST_FIRST(li)  ((li)->next)
#define LIST_LAST(li)   ((li)->prev)
#define LIST_END(li)    (li)
#define LIST_PREV(p)    ((p)->prev)
#define LIST_NEXT(p)    ((p)->next)

static struct list *
list_insert(struct list * const nd, struct list * const before)
{
  struct list *p = before->prev;

  nd->prev = p;
  nd->next = before;

  return (p->next = before->prev = nd);
}

static struct list *
list_erase(struct list * const nd)
{
  struct list *p = nd->prev, *q = nd->next;

  (p->next = q)->prev = p;

  nd->prev = nd->next = 0;

  return (nd);
}

struct pq_node {
  struct pq *pq;
  unsigned  idx;
};


enum {
  K9_OK          = 0,
  K9_INTERRUPTED = -1,
  K9_TIMED_OUT   = -2
};

enum {
  K9_OBJ_MAGIC_TASK  = 0x49395441, /* "K9TA" */
  K9_OBJ_MAGIC_EV    = 0x49394556, /* "K9EV" */
  K9_OBJ_MAGIC_MUTEX = 0x49394d55, /* "K9MU" */
  K9_OBJ_MAGIC_SEM   = 0x49395345  /* "K9SE" */
};

struct k9_obj {
  uint32        magic;
  struct k9_obj *parent;
  char          *id;
};

enum {
  K9_EV_FLAG_INTR_SAFE   = 1 << 0,
  K9_EV_FLAG_PRI_INHERIT = 1 << 1
};

struct k9_ev {
  struct k9_obj base[1];

  unsigned    flags;
  struct list list[1];
};
typedef struct k9_ev k9_ev[1];

struct k9_ev_wait_desc {
  struct k9_ev   *ev;
  unsigned       flag;
  struct list    list_node[1];
  struct k9_task *task;
};

enum {
  K9_TMOUT_NONE    = 0,
  K9_TMOUT_FOREVER = (unsigned) -1
};

struct k9_ev *k9_ev_init(struct k9_ev * const ev, char *id);
int          k9_ev_wait(unsigned nwait, struct k9_ev_wait_desc *wait, unsigned tmout);
int          k9_ev_wait1(struct k9_ev * const ev, unsigned tmout);
void         k9_ev_signal(struct k9_ev * const ev);
void         k9_ev_signal_all(struct k9_ev * const ev);

enum {
  K9_TASK_FLAG_SUSPENDED  = 1 << 0,
  K9_TASK_FLAG_NO_PREEMPT = 1 << 1
};

enum {
  K9_TASK_STATE_NEW,
  K9_TASK_STATE_READY,
  K9_TASK_STATE_RUNNING,
  K9_TASK_STATE_BLOCKED,
  K9_TASK_STATE_STOPPED,
  K9_TASK_STATE_EXITED,
  K9_TASK_STATE_FAULTED
};

struct k9_task {
  struct k9_obj base[1];

  uint8  flags, iflags;
  uint16 slice;
  int16  pri, effpri;
  void   *data;

  struct list list_node[1];

  uint8  state;
  int    exit_code;
  uint16 cur_slice;
  void   *sp;

  struct {
    unsigned ticks_blocked;
    unsigned ticks_ready;
    unsigned ticks_running;
  } stats[1];

  union {
    struct {
      unsigned               nwait;
      struct k9_ev_wait_desc *wait;
      struct {
	struct pq_node pq_node[1];
	unsigned       deadline;
      } tmout;
      int rc;
    } blocked;
    struct {
      struct pq_node pq_node[1];
    } ready;
  } u;
};
typedef struct k9_task k9_task[1];

struct k9_task *k9_task_init(struct k9_task *task, char *id, void *sp, void (*entry)(void *), void *arg);
struct k9_task *k9_task_start(struct k9_task *task);
void           k9_task_sleep(unsigned tmout);
void           k9_task_yield(void);
void           k9_task_exit(int code);
unsigned       k9_task_flags_set(struct k9_task *task, unsigned val, unsigned mask);
int            k9_task_pri_set(struct k9_task *task, int pri);
struct k9_task *k9_task_self(void);
void           k9_tick(void);
void           k9_isr(void (*func)(void *), void *arg);

struct k9_mutex {
  struct k9_ev   base[1];
  struct k9_task *owner;
};
typedef struct k9_mutex k9_mutex[1];

void k9_mutex_init(struct k9_mutex * const m, char *id);
int  k9_mutex_take(struct k9_mutex *m, unsigned tmout);
void k9_mutex_give(struct k9_mutex *m);

struct k9_sem {
  struct k9_ev base[1];
  int          cnt;
};
typedef struct k9_sem k9_sem[1];

void k9_sem_init(struct k9_sem * const s, char *id, int cnt);
int  k9_sem_take(struct k9_sem * const s, unsigned tmout);
void k9_sem_give(struct k9_sem * const s);

void k9_init(void);
void k9_start(struct k9_task *root_task, struct k9_task *idle_task, void *intr_stk_end);


