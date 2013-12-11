#include <assert.h>

#include "k9.h"

#define ARRAY_SIZE(a)  (sizeof(a) / sizeof((a)[0]))
#define END(a)         (&(a)[ARRAY_SIZE(a)])

#define FIELD_OFS(s, f)                   ((int)&((s *) 0)->f)
#define FIELD_PTR_TO_STRUCT_PTR(p, s, f)  ((s *)((char *)(p) - FIELD_OFS(s, f)))


static void
k9_obj_init(struct k9_obj *obj, unsigned magic, struct k9_obj *parent, char *id)
{
  obj->magic  = magic;
  obj->parent = parent;
  obj->id     = id;
}


struct pq;

struct pq {
  unsigned       size, cnt;
  int            (*cmp)(struct pq_node *, struct pq_node *);
  struct pq_node **arr;
};

static struct pq_node *
pq_node_init(struct pq_node *nd)
{
  nd->pq = 0;

  return (nd);
}

static struct pq_node *
pq_first(struct pq *pq)
{
  return (pq->cnt ? pq->arr[0] : 0);
}

static void
pq_swap(struct pq *pq, unsigned i, unsigned j)
{
  struct pq_node *temp;

  if (i == j)  return;

  temp = pq->arr[i];
  (pq->arr[i] = pq->arr[j])->idx = i;
  (pq->arr[j] = temp)->idx = j;
}

static void
pq_insert(struct pq *pq, struct pq_node *nd)
{
  unsigned i, j;

  assert(pq->cnt < pq->size);

  i = pq->cnt;
  (pq->arr[i] = nd)->idx = i;
  nd->pq = pq;

  for ( ; i != 0; i = j) {
    j = (i - 1) >> 1;

    if ((*pq->cmp)(pq->arr[i], pq->arr[j]) >= 0)  break;

    pq_swap(pq, i, j);
  }

  ++pq->cnt;
}

static void
pq_erase(struct pq_node *nd)
{
  struct pq      *pq;
  unsigned       i, j, k;
  struct pq_node *parent, *left, *right;
  int            cl, cr;

  pq = nd->pq;

  if (pq == 0)  return;

  assert(pq->cnt > 0);

  if (--pq->cnt == 0)  return;

  pq_swap(pq, i = nd->idx, pq->cnt);

  for (;;) {
    parent = pq->arr[i];
    if ((j = (i << 1) + 1) >= pq->cnt) {
      /* No children */

      break;
    }
    left = pq->arr[j];
    cl = (*pq->cmp)(parent, left);
    if ((k = j + 1) >= pq->cnt) {
      /* Left child only */

      if (cl < 0) {
	/* Left child OK */

	break;
      }

      /* Left child not OK */
      goto swap_left;
    }

    /* 2 children */

    right = pq->arr[k];
    cr = (*pq->cmp)(parent, right);
    if (cl < 0) {
      /* Left child OK */

      if (cr < 0) {
	/* Right child OK */

	break;
      }

      /* Left child OK and right child not OK */

      goto swap_right;
    }

    /* Left child not OK */

    if (cr < 0) {
      /* Right child OK */

      goto swap_left;
    }

    /* Left child not OK and right child not OK */

    if ((*pq->cmp)(left, right))  goto swap_left;

  swap_right:
    pq_swap(pq, i, k);
    i = k;
    continue;

  swap_left:
    pq_swap(pq, i, j);
    i = j;
  }
}

static void
pq_init(struct pq *pq, unsigned size, struct pq_node **arr, int (*cmp)(struct pq_node *, struct pq_node *))
{
  pq->size = size;
  pq->arr  = arr;
  pq->cmp  = cmp;
  pq->cnt  = 0;
}

static int
pq_node_task_pri_cmp(struct pq_node *nd1, struct pq_node *nd2)
{
  return (FIELD_PTR_TO_STRUCT_PTR(nd1, struct k9_task, u->ready->pq_node)->effpri
	  - FIELD_PTR_TO_STRUCT_PTR(nd2, struct k9_task, u->ready->pq_node)->effpri
	  );
}

static struct pq_node *rdy_pq_nodes[K9_CFG_MAX_TASKS];
static struct pq      rdy_pq[1];


static int
ticks_cmp(unsigned t1, unsigned t2)
{
  return ((int) t1 - (int) t2);
}

static int
pq_node_task_tmout_cmp(struct pq_node *nd1, struct pq_node *nd2)
{
  return (ticks_cmp(FIELD_PTR_TO_STRUCT_PTR(nd1, struct k9_task, u->blocked->tmout->pq_node)->u->blocked->tmout->deadline,
		    FIELD_PTR_TO_STRUCT_PTR(nd2, struct k9_task, u->blocked->tmout->pq_node)->u->blocked->tmout->deadline
		    )
	  );
}

static struct pq_node *tmout_pq_nodes[K9_CFG_MAX_TASKS];
static struct pq      tmout_pq[1];


extern uint32   k9_cpu_intr_dis(void);
extern void     k9_cpu_intr_restore(uint32);
extern uint32   k9_cpu_cntxt_save(void **);
extern void     k9_cpu_cntxt_restore(void *);
extern void     *k9_cpu_cntxt_init(void *stk_end, void (* entry)(void *), void *arg);
extern void     k9_cpu_intr_stk_end_set(void *stk_end);
extern void     k9_cpu_isr(void (*func)(void *), void *arg);
extern unsigned k9_cpu_intr_lvl(void);

static unsigned cur_ticks;

static struct list task_list[1];

static struct k9_task *cur_task, *idle_task;

static void task_unblock(struct k9_task *task, int block_rc);
static void task_ready(struct k9_task *task);
void _k9_task_resched(void);

static struct k9_cfg *k9_cfg;

static struct k9_task *
task_or_self(struct k9_task *task)
{
  return (task ? task : cur_task);
}

static void
tmout_insert(struct k9_task * const task, unsigned tmout)
{
  struct list *p;

  if (tmout == K9_TMOUT_FOREVER) {
    pq_node_init(task->u->blocked->tmout->pq_node);

    return;
  }
  
  task->u->blocked->tmout->deadline = cur_ticks + tmout;
  
  pq_insert(tmout_pq, task->u->blocked->tmout->pq_node);
}

static void
tmout_erase(struct k9_task * const task)
{
  pq_erase(task->u->blocked->tmout->pq_node);
}

static unsigned 
tmout_chk(void)
{
  unsigned       result = 0;
  struct pq_node *nd;

  while (nd = pq_first(tmout_pq)) {
    struct k9_task *task = FIELD_PTR_TO_STRUCT_PTR(nd, struct k9_task, u->blocked->tmout->pq_node);

    if (ticks_cmp(task->u->blocked->tmout->deadline, cur_ticks) > 0)  break;
        
    task_unblock(task, K9_TIMED_OUT);

    ++result;
  }

  return (result);
}

static void
task_ticks_update(void)
{
  struct list *p;

  for (p = LIST_FIRST(task_list); p != LIST_END(task_list); p = LIST_NEXT(p)) {
    struct k9_task *task = FIELD_PTR_TO_STRUCT_PTR(p, struct k9_task, list_node);
        
    switch (task->state) {
    case K9_TASK_STATE_RUNNING:
      ++task->stats->ticks->running;
      if (task->cur_slice != 0)  --task->cur_slice;
      break;
    case K9_TASK_STATE_READY:
      ++task->stats->ticks->ready;
      break;
    case K9_TASK_STATE_BLOCKED:
      ++task->stats->ticks->blocked;
      break;
    default:
      ;
    }
  }
}

void
k9_tick(void)
{
  uint32 old;

  old = k9_cpu_intr_dis();
  
  ++cur_ticks;

  task_ticks_update();

  if (tmout_chk())  _k9_task_resched();

  k9_cpu_intr_restore(old);
}

struct k9_task *
rdy_first(void)
{
  struct pq_node *nd;

  return ((nd = pq_first(rdy_pq)) ? FIELD_PTR_TO_STRUCT_PTR(nd, struct k9_task, u->ready->pq_node) : 0);
}

void
rdy_insert(struct k9_task *task)
{
  if (task == idle_task)  return;

  pq_insert(rdy_pq, task->u->ready->pq_node);
}

void
rdy_erase(struct k9_task *task)
{
  if (task == idle_task)  return;

  pq_erase(task->u->ready->pq_node);
}

enum {
  TASK_IFLAG_YIELD = 1 << 0
};

static struct k9_task *
task_select(void)
{
  struct k9_task *rdy_task = rdy_first();
  unsigned       yf;

  if (yf = cur_task->iflags & TASK_IFLAG_YIELD)  cur_task->iflags &= ~TASK_IFLAG_YIELD;

  if (cur_task->state != K9_TASK_STATE_RUNNING) {
    /* Current task no longer running

       => Must select new task; if none ready, use idle task
    */

    return (rdy_task ? rdy_task : idle_task);
  }
  
  if (rdy_task == 0) {
    /* Current task is running
       && no other task ready to run

       => Stay with current task
    */

    return (cur_task);
  }

  if (cur_task == idle_task) {
    /* Current task is running
       && other task ready to run
       && current task is idle task

       => Switch to ready task

       N.B.
       => Neither NO_PREEMPT nor priority has any meaning
       for idle task; it always loses in scheduling to any
       other ready task.
    */

    return (rdy_task);
  }

  if (cur_task->flags & K9_TASK_FLAG_NO_PREEMPT) {
    /* Current task is running
       && other task ready to run
       && current task is not idle task
       && current task is not pre-emptible

       => Stay with current task

       N.B.
       => NO_PREEMPT will preclude task switch, regardless of
       current tasks's priority, yielding or timeslice.
    */

    return (cur_task);
  }

  if (rdy_task->effpri != cur_task->effpri) {
    /* Current task is running
       && other task ready to run
       && current task is not idle task
       && current task is pre-emptible
       && ready task and current task have different priorities

       => Choose task with higher priority
    */

    return (rdy_task->effpri > cur_task->effpri ? rdy_task : cur_task);
  }

  if (yf || cur_task->cur_slice == 0) {
    /* Current task is running
       && other task ready to run
       && current task is not idle task
       && current task is pre-emptible
       && ready task and current task have same priority
       && (current task has yielded or consumed its timeslice)

       => Switch to ready task
    */

    return (rdy_task);
  }

  /* Current task is running
     && other task ready to run
     && current task is not idle task
     && current task is pre-emptible
     && ready task and current task have same priority
     && current task has not yielded
     && current task has not consumed its timeslice

     => Stay with current task
  */
    
  return (cur_task);
}

static void
task_switch(struct k9_task * const task)
{
  static struct k9_task *t;

  if (task == cur_task)  return;

  t = task;

  if (t->state == K9_TASK_STATE_READY)  rdy_erase(t);
  
  if (cur_task->state == K9_TASK_STATE_RUNNING)  task_ready(cur_task);
  
  if (k9_cpu_cntxt_save(&cur_task->sp) == 0)  return;
  
  (cur_task = t)->state = K9_TASK_STATE_RUNNING;
  cur_task->cur_slice   = cur_task->slice;

  k9_cpu_cntxt_restore(cur_task->sp);
}

void
_k9_task_resched(void)
{
  if (k9_cpu_intr_lvl() == 0)  task_switch(task_select());
}

static void
task_ready(struct k9_task * const task)
{
  task->state = K9_TASK_STATE_READY;

  if (task->flags & K9_TASK_FLAG_SUSPENDED)  return;
  
  rdy_insert(task);
}

static void
task_ev_wait_erase(struct k9_task * const task)
{
  struct k9_ev_wait_desc *w;
  unsigned                n;
  
  for (w = task->u->blocked->ev->wait, n = task->u->blocked->ev->nwait; n; --n, ++w) {
    list_erase(w->list_node);
  }
  
  tmout_erase(task);
}

static void
task_unblock(struct k9_task * const task, int block_rc)
{
  task_ev_wait_erase(task);
    
  task->u->blocked->rc = block_rc;

  task_ready(task);
}

static void
task_block(unsigned tmout)
{
  assert(k9_cpu_intr_lvl() == 0);
  assert(cur_task != idle_task);

  cur_task->state = K9_TASK_STATE_BLOCKED;

  tmout_insert(cur_task, tmout);

  _k9_task_resched();
}

static unsigned
task_stop(struct k9_task *task, unsigned new_state)
{
  unsigned old_state;

  switch (old_state = task->state) {
  case K9_TASK_STATE_RUNNING:
    task->state = new_state;
    break;
    
  case K9_TASK_STATE_READY:
    if (!(task->flags & K9_TASK_FLAG_SUSPENDED))  rdy_erase(task);
    task->state = new_state;
    break;

  case K9_TASK_STATE_BLOCKED:
    task_ev_wait_erase(task);
    task->state = new_state;
    break;

  default:
    ;
  }

  return (old_state == K9_TASK_STATE_RUNNING);
}

static unsigned
task_flags_set(struct k9_task * const task, unsigned val, unsigned mask)
{
  unsigned result    = 0;
  unsigned susp_prev = task->flags & K9_TASK_FLAG_SUSPENDED, susp = val & K9_TASK_FLAG_SUSPENDED;

  task->flags = (task->flags & ~mask) | val;

  if (susp != susp_prev) {
    if (susp) {
      switch (task->state) {
      case K9_TASK_STATE_RUNNING:
	task->state = K9_TASK_STATE_READY;
	result = 1;
	break;
      case K9_TASK_STATE_READY:
	rdy_erase(task);
	break;
      default:
	;
      }
    } else {
      switch (task->state) {
      case K9_TASK_STATE_READY:
	rdy_insert(task);
	result = 1;
	break;
      default:
	;
      }
    }
  }
  
  return (result);
}

static unsigned
task_effpri_set(struct k9_task * const task, int pri)
{
  unsigned result = 0;

  task->effpri = pri;

  switch (task->state) {
  case K9_TASK_STATE_RUNNING:
    result = 1;
    break;
  case K9_TASK_STATE_READY:
    rdy_erase(task);
    rdy_insert(task);
    result = 1;
    break;
  case K9_TASK_STATE_BLOCKED:
    {
      struct k9_ev_wait_desc *w;
      unsigned               n;

      for (w = task->u->blocked->ev->wait, n = task->u->blocked->ev->nwait; n; --n, ++w) {
	struct k9_ev *ev = w->ev;
	
	if (ev->base->magic == K9_OBJ_MAGIC_MUTEX
	    && (ev->flags & K9_EV_FLAG_PRI_INHERIT)
	    ) {
	  struct k9_task *owner = ((struct k9_mutex *) ev)->owner;
	  
	  if (owner && task->effpri > owner->effpri) {
	    result |= task_effpri_set(owner, task->effpri);
	  }
	}
      }
    }
    break;
  default:
    assert(0);
  }
    
  return (result);
}

static unsigned
task_pri_set(struct k9_task * const task, int pri)
{
  unsigned result = 0, prom = task->effpri > task->pri;

  task->pri = pri;

  if (prom && task->pri > task->effpri || !prom)  result = task_effpri_set(task, task->pri);

  return (result);
}

static void
ev_init(struct k9_ev *ev, unsigned magic, char *id)
{
  k9_obj_init(ev->base, magic, cur_task->base, id);

  LIST_INIT(ev->list);
}

static int
ev_wait(unsigned nwait, struct k9_ev_wait_desc *wait, unsigned tmout)
{
  cur_task->u->blocked->ev->nwait = nwait;
  cur_task->u->blocked->ev->wait  = wait;

  for ( ; nwait; --nwait, ++wait) {
    wait->flag = 0;
    list_insert(wait->list_node, LIST_END(wait->ev->list));
    wait->task = cur_task;
  }

  task_block(tmout);

  return (cur_task->u->blocked->rc);
}


static int
ev_wait1(struct k9_ev *ev, unsigned tmout)
{
  struct k9_ev_wait_desc wait[1];

  wait->ev = ev;

  return (ev_wait(1, wait, tmout));
}


static unsigned
ev_signal(struct k9_ev * const ev, unsigned cnt)
{
  unsigned    n;
  struct list *p;

  for (n = 0; (cnt == 0 || n < cnt) && (p = LIST_FIRST(ev->list)) != LIST_END(ev->list); ++n) {
    struct k9_ev_wait_desc *q = FIELD_PTR_TO_STRUCT_PTR(p, struct k9_ev_wait_desc, list_node);
    
    q->flag = 1;
    task_unblock(q->task, K9_OK);
  }
    
  return (n);
}

static struct k9_task *
ev_first(struct k9_ev * const ev)
{
  struct list *p;

  return (((p = LIST_FIRST(ev->list)) == LIST_END(ev->list)) ? 0 : FIELD_PTR_TO_STRUCT_PTR(p, struct k9_ev_wait_desc, list_node)->task);
}

/***************************************************************************/

struct k9_task *
k9_task_init(struct k9_task *task, char *id, void *sp, void (*entry)(void *), void *arg)
{
  k9_obj_init(task->base, K9_OBJ_MAGIC_TASK, cur_task->base, id);

  task->sp = k9_cpu_cntxt_init(sp, entry, arg);

  list_insert(task->list_node, LIST_END(task_list));
}


struct k9_task *
k9_task_start(struct k9_task *task)
{
  uint32 old;

  old = k9_cpu_intr_dis();
  
  task_ready(task);

  _k9_task_resched();

  k9_cpu_intr_restore(old);

  return (task);
}


void
k9_task_stop(struct k9_task *task)
{
  uint32  old;

  task = task_or_self(task);

  assert(task != idle_task);

  old = k9_cpu_intr_dis();

  if (task_stop(task, K9_TASK_STATE_STOPPED))  _k9_task_resched();

  k9_cpu_intr_restore(old);
}


void
k9_task_exit(int code)
{
  k9_cpu_intr_dis();

  assert(cur_task != idle_task);

  cur_task->exit_code = code;

  task_stop(cur_task, K9_TASK_STATE_EXITED);
  
  _k9_task_resched();
}


int
k9_task_delete(struct k9_task *task)
{
  uint32   old;
  unsigned f = 0;

  task = task_or_self(task);

  old = k9_cpu_intr_dis();

  switch (task->state) {
  case K9_TASK_STATE_STOPPED:
  case K9_TASK_STATE_EXITED:
  case K9_TASK_STATE_FAULTED:
    break;

  default:
    return (-1);
  }

  list_erase(task->list_node);

  k9_cpu_intr_restore(old);
}


struct k9_task *
k9_task_self(void)
{
  return (cur_task);
}


void
k9_task_sleep(unsigned tmout)
{
  uint32 old;

  old = k9_cpu_intr_dis();

  task_block(tmout);

  k9_cpu_intr_restore(old);
}


int 
k9_task_pri_set(struct k9_task *task, int pri)
{
  int    result;
  uint32 old;

  task = task_or_self(task);

  old = k9_cpu_intr_dis();

  result = task->pri;

  if (task_pri_set(task, pri))  _k9_task_resched();

  k9_cpu_intr_restore(old);

  return (result);
}


uint32
k9_task_flags_set(struct k9_task *task, uint32 val, uint32 mask)
{
  uint32 result, old;

  task = task_or_self(task);

  old = k9_cpu_intr_dis();

  result = task->flags;

  if (task_flags_set(task, val, mask))  _k9_task_resched();

  k9_cpu_intr_restore(old);

  return (result);
}


void
k9_task_yield(void)
{
  uint32 old;

  old = k9_cpu_intr_dis();

  cur_task->iflags |= TASK_IFLAG_YIELD;

  _k9_task_resched();

  k9_cpu_intr_restore(old);
}


struct k9_ev *
k9_ev_init(struct k9_ev *ev, char *id)
{
  ev_init(ev, K9_OBJ_MAGIC_EV, id);
}


int
k9_ev_wait(unsigned nwait, struct k9_ev_wait_desc *wait, unsigned tmout)
{
  int    result;
  uint32 old;

  old = k9_cpu_intr_dis();

  result = ev_wait(nwait, wait, tmout);

  k9_cpu_intr_restore(old);

  return (result);
}


int
k9_ev_wait1(struct k9_ev *ev, unsigned tmout)
{
  int    result;
  uint32 old;

  old = k9_cpu_intr_dis();

  result = ev_wait1(ev, tmout);

  k9_cpu_intr_restore(old);

  return (result);
}


void
k9_ev_signal(struct k9_ev * const ev)
{
  uint32 old;

  old = k9_cpu_intr_dis();

  if (ev_signal(ev, 1) != 0)  _k9_task_resched();

  k9_cpu_intr_restore(old);
}

void
k9_ev_signal_all(struct k9_ev * const ev)
{
  uint32 old;

  old = k9_cpu_intr_dis();

  if (ev_signal(ev, 0) != 0)  _k9_task_resched();

  k9_cpu_intr_restore(old);
}


void
k9_mutex_init(struct k9_mutex * const m, char *id)
{
  ev_init(m->base, K9_OBJ_MAGIC_MUTEX, id);

  m->owner = 0;
}


int
k9_mutex_take(struct k9_mutex * const m, unsigned tmout)
{
  int    result = K9_OK;
  uint32 old;

  old = k9_cpu_intr_dis();

  if (m->owner != 0) {
    if (tmout == K9_TMOUT_NONE) {
      result = K9_TIMED_OUT;
    } else {
      struct k9_ev *ev = m->base;

      if (ev->flags & K9_EV_FLAG_PRI_INHERIT) {
	struct k9_task *owner = m->owner;

	if (cur_task->effpri > owner->effpri)  task_effpri_set(owner, cur_task->effpri);
      }

      result = ev_wait1(ev, tmout);
    }
  }

  if (result == K9_OK)  m->owner = cur_task;

  k9_cpu_intr_restore(old);

  return (result);
}


void
k9_mutex_give(struct k9_mutex * const m)
{
  uint32   old;
  unsigned n1 = 0, n2 = 0;

  old = k9_cpu_intr_dis();

  if (m->owner == cur_task) {
    m->owner = 0;

    n1 = ev_signal(m->base, 1);

    if (cur_task->effpri != cur_task->pri)  n2 = task_effpri_set(cur_task, cur_task->pri);

    if (n1 != 0 || n2 != 0)  _k9_task_resched;
  }

  k9_cpu_intr_restore(old);
}


void
k9_sem_init(struct k9_sem * const s, char *id, int cnt)
{
  ev_init(s->base, K9_OBJ_MAGIC_SEM, id);

  s->cnt = cnt;
}

static void
sem_chk_first(struct k9_sem * const s)
{
  struct k9_task *t;
  
  if ((t = ev_first(s->base)) && t->u->blocked->sem->n <= s->cnt) {
    ev_signal(s->base, 1);
  }
}

int
k9_sem_take(struct k9_sem * const s, unsigned n, unsigned tmout)
{
  int    result = K9_OK;
  uint32 old;

  old = k9_cpu_intr_dis();

  if (ev_first(s->base) != 0 || n > s->cnt) {
    if (tmout == K9_TMOUT_NONE) {
      result = K9_TIMED_OUT;
    } else {
      cur_task->u->blocked->sem->n = n;

      switch (result = ev_wait1(s->base, tmout)) {
      case K9_OK:
	break;

      case K9_TIMED_OUT:
	sem_chk_first(s);
	break;

      default:
	assert(0);
      }
    }
  }

  if (result == K9_OK)  s->cnt -= (int) n;

  k9_cpu_intr_restore(old);

  return (result);
}


void
k9_sem_give(struct k9_sem * const s, unsigned n)
{
  uint32 old;

  old = k9_cpu_intr_dis();

  s->cnt += (int) n;

  sem_chk_first(s);

  k9_cpu_intr_restore(old);
}


void
k9_init(void)
{
  pq_init(rdy_pq, ARRAY_SIZE(rdy_pq_nodes), rdy_pq_nodes, pq_node_task_pri_cmp);
  pq_init(tmout_pq, ARRAY_SIZE(tmout_pq_nodes), tmout_pq_nodes, pq_node_task_tmout_cmp);

  LIST_INIT(task_list);
}


void
k9_start(struct k9_task *root_task, struct k9_task *_idle_task, void *intr_stk_end)
{
  idle_task = _idle_task;

  k9_cpu_intr_stk_end_set(intr_stk_end);

  (cur_task = root_task)->state = K9_TASK_STATE_RUNNING;
  cur_task->cur_slice = cur_task->slice;

  k9_cpu_cntxt_restore(cur_task->sp);
}

/***************************************************************************/

#ifdef __UNIT_TEST__

#include <stdlib.h>
#include <stdio.h>

#define PRINT_EXPR(f, x)  printf("%s = " f, #x, (x))

void
test_idle_main(void *arg)
{
  printf("%s\n", "test_idle task starting...");

  for (;;) {
    printf("%s\n", "test_idle loop");

#if 0
    k9_cpu_wait();
#else
    k9_cpu_isr((void (*)(void *)) k9_tick, 0);

    if (cur_ticks >= 100)  exit(0);
#endif
  }
  printf("%s\n", "test_idle task exiting");
}

struct k9_sem test_sem[1];

void
test_appl_main(void *arg)
{
  unsigned n = (unsigned) arg;
  int      rc;

  printf("%s\n", "test_appl task starting...");

  rc = k9_sem_take(test_sem, n, n == 10 ? 10 : K9_TMOUT_FOREVER);

  printf("test_appl: ");
  PRINT_EXPR("%d", rc);
  printf("\n");

  printf("%s\n", "test_appl task exiting");
}

struct k9_task test_appl_task[2];
unsigned char test_appl_stk[2][2048];

struct {
  struct k9_task *task;
  char           *id;
  unsigned char  *stk_top;
  void           (*entry)(void *);
  void           *arg;
} task_init_tbl[] = {
  { &test_appl_task[0], "task1", END(test_appl_stk[0]), test_appl_main, (void *) 10 },
  { &test_appl_task[1], "task2", END(test_appl_stk[1]), test_appl_main, (void *) 1 }
};


void
test_root_main(void *arg)
{
  unsigned i;

  printf("%s\n", "test_root task starting...");
  printf("%s\n", (char *) arg);

  k9_sem_init(test_sem, "sem1", 5);

  for (i = 0; i < ARRAY_SIZE(task_init_tbl); ++i) {
    k9_task_init(task_init_tbl[i].task, task_init_tbl[i].id, task_init_tbl[i].stk_top, task_init_tbl[i].entry, task_init_tbl[i].arg);
    task_init_tbl[i].task->slice = 100;
  }

  for (i = 0; i < ARRAY_SIZE(task_init_tbl); ++i) {
    k9_task_start(task_init_tbl[i].task);
  }

  printf("%s\n", "test_root task exiting");

}

struct k9_task test_root_task[1], test_idle_task[1];
unsigned char test_root_stk[4096], test_idle_stk[1024], test_intr_stk[4096];

int
main(void)
{
  k9_init();
  k9_task_init(test_root_task, "root", END(test_root_stk), test_root_main, "test_root arg");
  test_root_task->slice = 100;
  k9_task_init(test_idle_task, "idle", END(test_idle_stk), test_idle_main, 0);
  k9_start(test_root_task, test_idle_task, END(test_intr_stk));

  return (0);
}

#endif /* __UNIT_TEST__ */
