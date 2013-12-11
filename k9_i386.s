	.bss

_intr_lvl:	.space	4
_intr_sp_save:	.space	4
_intr_stk_end:	.space	4
	
	.text

	.global	k9_cpu_intr_stk_end_set
k9_cpu_intr_stk_end_set:
	movl	4(%esp),%eax
	movl	%eax,_intr_stk_end
	ret
	
_intr_cntxt_leave:
	movl	_intr_sp_save,%esp
	ret	
	
_intr_cntxt_enter:
	movl	%esp,%eax
	movl	%eax,_intr_sp_save
	movl	_intr_stk_end,%esp
	pushl	12(%eax)
	pushl	$_intr_cntxt_leave
	pushl	8(%eax)
	ret

	.global	k9_cpu_isr
k9_cpu_isr:	
	movl	_intr_lvl,%eax
	incl	%eax
	movl	%eax,_intr_lvl
	cmpl	$1,%eax
	jne	k9_cpu_isr2
	call	_intr_cntxt_enter
	jmp	k9_cpu_isr3
k9_cpu_isr2:
	movl	8(%esp),%eax
	pushl	%eax
	movl	8(%esp),%eax
	call	*%eax
	lea	4(%esp),%esp
k9_cpu_isr3:
#	cli			# In case called ISR re-enabled interrupts
	movl	_intr_lvl,%eax
	decl	%eax
	movl	%eax,_intr_lvl
	call	_k9_task_resched
	ret

	.global	k9_cpu_intr_lvl
k9_cpu_intr_lvl:	
	movl	_intr_lvl,%eax
	ret
	
_task_exit:
	pushl	$0
	call	k9_task_exit

	.global	k9_cpu_cntxt_init
k9_cpu_cntxt_init:
	pushl	%ebx
	movl	8(%esp),%eax
	lea	(12*-4)(%eax),%eax
	movl	12(%esp),%ebx
	movl	%ebx,(9*4)(%eax)
	movl	$_task_exit,(10*4)(%eax)
	movl	16(%esp),%ebx
	movl	%ebx,(11*4)(%eax)
	popl	%ebx
	ret
	
	.global	k9_cpu_cntxt_save
k9_cpu_cntxt_save:
	pushf
	pusha
	movl	(10*4)(%esp),%eax
	movl	%esp,(%eax)
	movl	(9*4)(%esp),%eax
	jmp	*%eax

	.global	k9_cpu_cntxt_restore
k9_cpu_cntxt_restore:
	movl	4(%esp),%esp
	popa
	popf
	xorl	%eax,%eax
	ret

	.global k9_cpu_intr_dis
k9_cpu_intr_dis:
	pushf
#	cli
	popl	%eax
	ret

	.global k9_cpu_intr_restore
k9_cpu_intr_restore:
#	movl	4(%esp),%eax
#	pushl	%eax
#	popf
	ret

	.global	k9_cpu_wait
k9_cpu_wait:
#	hlt
	ret