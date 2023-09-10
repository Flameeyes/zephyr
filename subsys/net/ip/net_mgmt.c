/*
 * Copyright (c) 2016 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(net_mgmt, CONFIG_NET_MGMT_EVENT_LOG_LEVEL);

#include <zephyr/kernel.h>
#include <zephyr/toolchain.h>
#include <zephyr/linker/sections.h>

#include <zephyr/sys/util.h>
#include <zephyr/sys/slist.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/debug/stack.h>

#include "net_private.h"

struct mgmt_event_entry {
#ifdef CONFIG_NET_MGMT_EVENT_INFO
	uint8_t info[NET_EVENT_INFO_MAX_SIZE];
	size_t info_length;
#endif /* CONFIG_NET_MGMT_EVENT_INFO */
	uint32_t event;
	struct net_if *iface;
};

BUILD_ASSERT((sizeof(struct mgmt_event_entry) % sizeof(uint32_t)) == 0,
	     "The structure must be a multiple of sizeof(uint32_t)");

struct mgmt_event_wait {
	struct k_sem sync_call;
	struct net_if *iface;
};

static K_MUTEX_DEFINE(net_mgmt_callback_lock);
static K_MUTEX_DEFINE(net_mgmt_event_lock);

K_KERNEL_STACK_DEFINE(mgmt_stack, CONFIG_NET_MGMT_EVENT_STACK_SIZE);
static struct k_thread mgmt_thread_data;
static uint32_t global_event_mask;
static sys_slist_t event_callbacks = SYS_SLIST_STATIC_INIT(&event_callbacks);

/* event structure used to prevent increasing the stack usage on the caller thread */
static struct mgmt_event_entry new_event;
K_MSGQ_DEFINE(event_msgq, sizeof(struct mgmt_event_entry),
	      CONFIG_NET_MGMT_EVENT_QUEUE_SIZE, sizeof(uint32_t));


static inline void mgmt_push_event(uint32_t mgmt_event, struct net_if *iface,
				   const void *info, size_t length)
{
#ifndef CONFIG_NET_MGMT_EVENT_INFO
	ARG_UNUSED(info);
	ARG_UNUSED(length);
#endif /* CONFIG_NET_MGMT_EVENT_INFO */

	(void)k_mutex_lock(&net_mgmt_event_lock, K_FOREVER);

	memset(&new_event, 0, sizeof(struct mgmt_event_entry));

#ifdef CONFIG_NET_MGMT_EVENT_INFO
	if (info && length) {
		if (length <= NET_EVENT_INFO_MAX_SIZE) {
			memcpy(new_event.info, info, length);
			new_event.info_length = length;
		} else {
			NET_ERR("Event %u info length %zu > max size %zu",
				mgmt_event, length, NET_EVENT_INFO_MAX_SIZE);
			(void)k_mutex_unlock(&net_mgmt_event_lock);

			return;
		}
	}
#endif /* CONFIG_NET_MGMT_EVENT_INFO */

	new_event.event = mgmt_event;
	new_event.iface = iface;

	if (k_msgq_put(&event_msgq, &new_event,
		K_MSEC(CONFIG_NET_MGMT_EVENT_QUEUE_TIMEOUT)) != 0) {
		NET_WARN("Failure to push event (%u), "
			 "try increasing the 'CONFIG_NET_MGMT_EVENT_QUEUE_SIZE' "
			 "or 'CONFIG_NET_MGMT_EVENT_QUEUE_TIMEOUT' options.",
			 mgmt_event);
	}

	(void)k_mutex_unlock(&net_mgmt_event_lock);
}

static inline void mgmt_pop_event(struct mgmt_event_entry *dst)
{
	do {
	} while (k_msgq_get(&event_msgq, dst, K_FOREVER) != 0);
}

static inline void mgmt_add_event_mask(uint32_t event_mask)
{
	global_event_mask |= event_mask;
}

static inline void mgmt_rebuild_global_event_mask(void)
{
	struct net_mgmt_event_callback *cb, *tmp;

	global_event_mask = 0U;

	SYS_SLIST_FOR_EACH_CONTAINER_SAFE(&event_callbacks, cb, tmp, node) {
		mgmt_add_event_mask(cb->event_mask);
	}
}

static inline bool mgmt_is_event_handled(uint32_t mgmt_event)
{
	return (((NET_MGMT_GET_LAYER(mgmt_event) &
		  NET_MGMT_GET_LAYER(global_event_mask)) ==
		 NET_MGMT_GET_LAYER(mgmt_event)) &&
		((NET_MGMT_GET_LAYER_CODE(mgmt_event) &
		  NET_MGMT_GET_LAYER_CODE(global_event_mask)) ==
		 NET_MGMT_GET_LAYER_CODE(mgmt_event)) &&
		((NET_MGMT_GET_COMMAND(mgmt_event) &
		  NET_MGMT_GET_COMMAND(global_event_mask)) ==
		 NET_MGMT_GET_COMMAND(mgmt_event)));
}

static inline void mgmt_run_callbacks(const struct mgmt_event_entry * const mgmt_event)
{
	sys_snode_t *prev = NULL;
	struct net_mgmt_event_callback *cb, *tmp;

	NET_DBG("Event layer %u code %u cmd %u",
		NET_MGMT_GET_LAYER(mgmt_event->event),
		NET_MGMT_GET_LAYER_CODE(mgmt_event->event),
		NET_MGMT_GET_COMMAND(mgmt_event->event));

	SYS_SLIST_FOR_EACH_CONTAINER_SAFE(&event_callbacks, cb, tmp, node) {
		if (!(NET_MGMT_GET_LAYER(mgmt_event->event) ==
		      NET_MGMT_GET_LAYER(cb->event_mask)) ||
		    !(NET_MGMT_GET_LAYER_CODE(mgmt_event->event) ==
		      NET_MGMT_GET_LAYER_CODE(cb->event_mask)) ||
		    (NET_MGMT_GET_COMMAND(mgmt_event->event) &&
		     NET_MGMT_GET_COMMAND(cb->event_mask) &&
		     !(NET_MGMT_GET_COMMAND(mgmt_event->event) &
		       NET_MGMT_GET_COMMAND(cb->event_mask)))) {
			continue;
		}

#ifdef CONFIG_NET_MGMT_EVENT_INFO
		if (mgmt_event->info_length) {
			cb->info = (void *)mgmt_event->info;
			cb->info_length = mgmt_event->info_length;
		} else {
			cb->info = NULL;
			cb->info_length = 0;
		}
#endif /* CONFIG_NET_MGMT_EVENT_INFO */

		if (NET_MGMT_EVENT_SYNCHRONOUS(cb->event_mask)) {
			struct mgmt_event_wait *sync_data =
				CONTAINER_OF(cb->sync_call,
					     struct mgmt_event_wait, sync_call);

			if (sync_data->iface &&
			    sync_data->iface != mgmt_event->iface) {
				continue;
			}

			NET_DBG("Unlocking %p synchronous call", cb);

			cb->raised_event = mgmt_event->event;
			sync_data->iface = mgmt_event->iface;

			sys_slist_remove(&event_callbacks, prev, &cb->node);

			k_sem_give(cb->sync_call);
		} else {
			NET_DBG("Running callback %p : %p",
				cb, cb->handler);

			cb->handler(cb, mgmt_event->event, mgmt_event->iface);
			prev = &cb->node;
		}
	}

#ifdef CONFIG_NET_DEBUG_MGMT_EVENT_STACK
	log_stack_usage(&mgmt_thread_data);
#endif
}

static void mgmt_thread(void)
{
	struct mgmt_event_entry mgmt_event;

	while (1) {
		mgmt_pop_event(&mgmt_event);

		NET_DBG("Handling events, forwarding it relevantly");

		/* take the lock to prevent changes to the callback structure during use */
		(void)k_mutex_lock(&net_mgmt_callback_lock, K_FOREVER);

		mgmt_run_callbacks(&mgmt_event);

		(void)k_mutex_unlock(&net_mgmt_callback_lock);

		/* forcefully give up our timeslot, to give time to the callback */
		k_yield();
	}
}

static int mgmt_event_wait_call(struct net_if *iface,
				uint32_t mgmt_event_mask,
				uint32_t *raised_event,
				struct net_if **event_iface,
				const void **info,
				size_t *info_length,
				k_timeout_t timeout)
{
	struct mgmt_event_wait sync_data = {
		.sync_call = Z_SEM_INITIALIZER(sync_data.sync_call, 0, 1),
	};
	struct net_mgmt_event_callback sync = {
		.sync_call = &sync_data.sync_call,
		.event_mask = mgmt_event_mask | NET_MGMT_SYNC_EVENT_BIT,
	};
	int ret;

	if (iface) {
		sync_data.iface = iface;
	}

	NET_DBG("Synchronous event 0x%08x wait %p", sync.event_mask, &sync);

	net_mgmt_add_event_callback(&sync);

	ret = k_sem_take(sync.sync_call, timeout);
	if (ret < 0) {
		if (ret == -EAGAIN) {
			ret = -ETIMEDOUT;
		}

		net_mgmt_del_event_callback(&sync);
		return ret;
	}

	if (raised_event) {
		*raised_event = sync.raised_event;
	}

	if (event_iface) {
		*event_iface = sync_data.iface;
	}

#ifdef CONFIG_NET_MGMT_EVENT_INFO
	if (info) {
		*info = sync.info;

		if (info_length) {
			*info_length = sync.info_length;
		}
	}
#endif /* CONFIG_NET_MGMT_EVENT_INFO */

	return ret;
}

void net_mgmt_add_event_callback(struct net_mgmt_event_callback *cb)
{
	NET_DBG("Adding event callback %p", cb);

	(void)k_mutex_lock(&net_mgmt_callback_lock, K_FOREVER);

	sys_slist_prepend(&event_callbacks, &cb->node);

	mgmt_add_event_mask(cb->event_mask);

	(void)k_mutex_unlock(&net_mgmt_callback_lock);
}

void net_mgmt_del_event_callback(struct net_mgmt_event_callback *cb)
{
	NET_DBG("Deleting event callback %p", cb);

	(void)k_mutex_lock(&net_mgmt_callback_lock, K_FOREVER);

	sys_slist_find_and_remove(&event_callbacks, &cb->node);

	mgmt_rebuild_global_event_mask();

	(void)k_mutex_unlock(&net_mgmt_callback_lock);
}

void net_mgmt_event_notify_with_info(uint32_t mgmt_event, struct net_if *iface,
				     const void *info, size_t length)
{
	if (mgmt_is_event_handled(mgmt_event)) {
		NET_DBG("Notifying Event layer %u code %u type %u",
			NET_MGMT_GET_LAYER(mgmt_event),
			NET_MGMT_GET_LAYER_CODE(mgmt_event),
			NET_MGMT_GET_COMMAND(mgmt_event));

		mgmt_push_event(mgmt_event, iface, info, length);
	}
}

int net_mgmt_event_wait(uint32_t mgmt_event_mask,
			uint32_t *raised_event,
			struct net_if **iface,
			const void **info,
			size_t *info_length,
			k_timeout_t timeout)
{
	return mgmt_event_wait_call(NULL, mgmt_event_mask,
				    raised_event, iface, info, info_length,
				    timeout);
}

int net_mgmt_event_wait_on_iface(struct net_if *iface,
				 uint32_t mgmt_event_mask,
				 uint32_t *raised_event,
				 const void **info,
				 size_t *info_length,
				 k_timeout_t timeout)
{
	NET_ASSERT(NET_MGMT_ON_IFACE(mgmt_event_mask));
	NET_ASSERT(iface);

	return mgmt_event_wait_call(iface, mgmt_event_mask,
				    raised_event, NULL, info, info_length,
				    timeout);
}

void net_mgmt_event_init(void)
{
#if defined(CONFIG_NET_TC_THREAD_COOPERATIVE)
/* Lowest priority cooperative thread */
#define THREAD_PRIORITY K_PRIO_COOP(CONFIG_NUM_COOP_PRIORITIES - 1)
#else
#define THREAD_PRIORITY K_PRIO_PREEMPT(CONFIG_NUM_PREEMPT_PRIORITIES - 1)
#endif

	k_thread_create(&mgmt_thread_data, mgmt_stack,
			K_KERNEL_STACK_SIZEOF(mgmt_stack),
			(k_thread_entry_t)mgmt_thread, NULL, NULL, NULL,
			THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&mgmt_thread_data, "net_mgmt");

	NET_DBG("Net MGMT initialized: queue of %u entries, stack size of %u",
		CONFIG_NET_MGMT_EVENT_QUEUE_SIZE,
		CONFIG_NET_MGMT_EVENT_STACK_SIZE);
}
