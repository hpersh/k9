	.bss

intr_sp_save:	.space	4
	
	.text

	.global	k9_cpu_intr_cntxt_enter
k9_cpu_intr_cntxt_enter:
	movl	%esp,%eax
	movl	%eax,intr_sp_save
	movl	k9_intr_stk_end,%esp
	jmpl	*(%eax)
	
	.global	k9_cpu_intr_cntxt_leave
k9_cpu_intr_cntxt_leave:
	movl	intr_sp_save,%esp
	xorl	%eax,%eax
	ret	
	
	.global	k9_cpu_cntxt_init
k9_cpu_cntxt_init:
	pushl	%ebx
	movl	8(%esp),%eax
	lea	(12*-4)(%eax),%eax
	movl	12(%esp),%ebx
	movl	%ebx,(9*4)(%eax)
	movl	$k9_task_exit,(10*4)(%eax)
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