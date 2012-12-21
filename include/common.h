#ifndef __COMMON_H_
#define __COMMON_H_

#include <stdio.h>

#define ARRAY_SIZE(x)	(sizeof((x))/sizeof((x)[0]))
#define ARG_NR(type, ...)  (sizeof((type[]){__VA_ARGS__})/sizeof(type))

#define NAME_SIZE	256
typedef int  		bool;
#define true		1
#define false		0

#define ERR		-1

#define __pr(fmt, ...) do { \
	fprintf(stderr, fmt, ##__VA_ARGS__); \
} while(0)

#define pr_warn(fmt, ...) __pr("WARN: "fmt, ##__VA_ARGS__)
#define pr_err(fmt, ...) __pr("ERROR: "fmt, ##__VA_ARGS__)

#define pr_warn_str(str) pr_warn("%s\n", str)
#define pr_err_str(str) pr_err("%s\n", str)

#ifdef DEBUG
#define dbg  __pr
/* debug with line position*/
#define dbgl(fmt, ...) dbg("DEBUG %s,%d: "fmt, __FILE__, __LINE__, ##__VA_ARGS__)
#else
#define dbg
#define dbgl
#endif

#define likely(x)      __builtin_expect(!!(x), 1)
#define unlikely(x)    __builtin_expect(!!(x), 0)

#define EXIT_BUG	-2
#define EXIT_LCALL	-3

#define BUG() do {							    \
	__pr("BUG failure at %s:%d/%s()!\n", __FILE__, __LINE__, __func__); \
        exit(EXIT_BUG);							\
} while(0)

#define BUG_ON(cond) do { if(unlikely(cond)) BUG(); } while(0)

/* lib function call */
#define __LCALL(call) do {				\
	err(EXIT_LCALL, "libcall " #call		\
		"BUG failure at %s:%d/%s()!\n",		\
		__FILE__, __LINE__, __func__);	\
} while(0)

#define LCALL(call) do {				\
	if(unlikely((call)))				\
		__LCALL(call);			\
} while(0)

#define __LCALL_RET(ret, call, cond) ({	\
	ret = (call);			\
	if (unlikely(cond)) \
		__LCALL(call); \
	ret; \
})

#define LCALL1(ret, call, errcode1) \
	__LCALL_RET(ret, call, ret && ret != -(errcode1))

#define RWCALL(ret, call)			\
	__LCALL_RET(ret, call, ret < 0)

#define RWCALL1(ret, call, errcode1)		\
	__LCALL_RET(ret, call, ret < 0 && ret != -(errcode1))

#endif
