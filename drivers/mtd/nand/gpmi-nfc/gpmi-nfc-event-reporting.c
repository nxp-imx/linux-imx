/*
 * Freescale GPMI NFC NAND Flash Driver
 *
 * Copyright (C) 2010 Freescale Semiconductor, Inc.
 * Copyright (C) 2008 Embedded Alley Solutions, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include "gpmi-nfc.h"

#if defined(EVENT_REPORTING)

/*
 * This variable and module parameter controls whether the driver reports event
 * information by printing to the console.
 */

static int report_events;
module_param(report_events, int, 0600);

/**
 * struct event - A single record in the event trace.
 *
 * @time:         The time at which the event occurred.
 * @nesting:      Indicates function call nesting.
 * @description:  A description of the event.
 */

struct event {
	ktime_t       time;
	unsigned int  nesting;
	char          *description;
};

/**
 * The event trace.
 *
 * @overhead:  The delay to take a time stamp and nothing else.
 * @nesting:   The current nesting level.
 * @overflow:  Indicates the trace overflowed.
 * @next:      Index of the next event to write.
 * @events:    The array of events.
 */

#define MAX_EVENT_COUNT  (200)

static struct {
	ktime_t       overhead;
	int           nesting;
	int           overflow;
	unsigned int  next;
	struct event  events[MAX_EVENT_COUNT];
} event_trace;

/**
 * gpmi_nfc_reset_event_trace() - Resets the event trace.
 */
void gpmi_nfc_reset_event_trace(void)
{
	event_trace.nesting  = 0;
	event_trace.overflow = false;
	event_trace.next     = 0;
}

/**
 * gpmi_nfc_add_event() - Adds an event to the event trace.
 *
 * @description:  A description of the event.
 * @delta:        A delta to the nesting level for this event [-1, 0, 1].
 */
void gpmi_nfc_add_event(char *description, int delta)
{
	struct event  *event;

	if (!report_events)
		return;

	if (event_trace.overflow)
		return;

	if (event_trace.next >= MAX_EVENT_COUNT) {
		event_trace.overflow = true;
		return;
	}

	event = event_trace.events + event_trace.next;

	event->time = ktime_get();

	event->description = description;

	if (!delta)
		event->nesting = event_trace.nesting;
	else if (delta < 0) {
		event->nesting = event_trace.nesting - 1;
		event_trace.nesting -= 2;
	} else {
		event->nesting = event_trace.nesting + 1;
		event_trace.nesting += 2;
	}

	if (event_trace.nesting < 0)
		event_trace.nesting = 0;

	event_trace.next++;

}

/**
 * gpmi_nfc_start_event_trace() - Starts an event trace.
 *
 * @description:  A description of the first event.
 */
void gpmi_nfc_start_event_trace(char *description)
{

	ktime_t  t0;
	ktime_t  t1;

	if (!report_events)
		return;

	gpmi_nfc_reset_event_trace();

	t0 = ktime_get();
	t1 = ktime_get();

	event_trace.overhead = ktime_sub(t1, t0);

	gpmi_nfc_add_event(description, 1);

}

/**
 * gpmi_nfc_dump_event_trace() - Dumps the event trace.
 */
void gpmi_nfc_dump_event_trace(void)
{
	unsigned int  i;
	time_t        seconds;
	long          nanoseconds;
	char          line[100];
	int           o;
	struct event  *first_event;
	struct event  *last_event;
	struct event  *matching_event;
	struct event  *event;
	ktime_t       delta;

	/* Check if event reporting is turned off. */

	if (!report_events)
		return;

	/* Print important facts about this event trace. */

	pr_info("\n+----------------\n");

	pr_info("|  Overhead    : [%d:%d]\n", event_trace.overhead.tv.sec,
						event_trace.overhead.tv.nsec);

	if (!event_trace.next) {
		pr_info("|  No Events\n");
		return;
	}

	first_event = event_trace.events;
	last_event  = event_trace.events + (event_trace.next - 1);

	delta = ktime_sub(last_event->time, first_event->time);
	pr_info("|  Elapsed Time: [%d:%d]\n", delta.tv.sec, delta.tv.nsec);

	if (event_trace.overflow)
		pr_info("|  Overflow!\n");

	/* Print the events in this history. */

	for (i = 0, event = event_trace.events;
					i < event_trace.next; i++, event++) {

		/* Get the delta between this event and the previous event. */

		if (!i) {
			seconds     = 0;
			nanoseconds = 0;
		} else {
			delta = ktime_sub(event[0].time, event[-1].time);
			seconds     = delta.tv.sec;
			nanoseconds = delta.tv.nsec;
		}

		/* Print the current event. */

		o = 0;

		o = snprintf(line, sizeof(line) - o, "|  [%ld:% 10ld]%*s %s",
							seconds, nanoseconds,
							event->nesting, "",
							event->description);
		/* Check if this is the last event in a nested series. */

		if (i && (event[0].nesting < event[-1].nesting)) {

			for (matching_event = event - 1;; matching_event--) {

				if (matching_event < event_trace.events) {
					matching_event = 0;
					break;
				}

				if (matching_event->nesting == event->nesting)
					break;

			}

			if (matching_event) {
				delta = ktime_sub(event->time,
							matching_event->time);
				o += snprintf(line + o, sizeof(line) - o,
						" <%d:%d]", delta.tv.sec,
								delta.tv.nsec);
			}

		}

		/* Check if this is the first event in a nested series. */

		if ((i < event_trace.next - 1) &&
				(event[0].nesting < event[1].nesting)) {

			for (matching_event = event + 1;; matching_event++) {

				if (matching_event >=
					(event_trace.events+event_trace.next)) {
					matching_event = 0;
					break;
				}

				if (matching_event->nesting == event->nesting)
					break;

			}

			if (matching_event) {
				delta = ktime_sub(matching_event->time,
								event->time);
				o += snprintf(line + o, sizeof(line) - o,
						" [%d:%d>", delta.tv.sec,
								delta.tv.nsec);
			}

		}

		pr_info("%s\n", line);

	}

	pr_info("+----------------\n");

}

/**
 * gpmi_nfc_stop_event_trace() - Stops an event trace.
 *
 * @description:  A description of the last event.
 */
void gpmi_nfc_stop_event_trace(char *description)
{
	struct event  *event;

	if (!report_events)
		return;

	/*
	 * We want the end of the trace, no matter what happens. If the trace
	 * has already overflowed, or is about to, just jam this event into the
	 * last spot. Otherwise, add this event like any other.
	 */

	if (event_trace.overflow || (event_trace.next >= MAX_EVENT_COUNT)) {
		event = event_trace.events + (MAX_EVENT_COUNT - 1);
		event->time = ktime_get();
		event->description = description;
		event->nesting     = 0;
	} else {
		gpmi_nfc_add_event(description, -1);
	}

	gpmi_nfc_dump_event_trace();
	gpmi_nfc_reset_event_trace();

}

#endif /* EVENT_REPORTING */
