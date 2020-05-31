/*
 * Generic show_mem() implementation
 *
 * Copyright (C) 2008 Johannes Weiner <hannes@saeurebad.de>
 * All code subject to the GPL version 2.
 */

#include <linux/mm.h>
#include <linux/quicklist.h>
#include <linux/cma.h>

#if defined(CONFIG_LAB126)
#include <linux/swap.h>
#endif

#if defined(CONFIG_LAB126)
unsigned int rss_sum_by_tasks(void)
{
	unsigned int rss_sum = 0;
	struct task_struct *g, *p;
	do_each_thread(g, p) {
		struct mm_struct *mm;

		if (!thread_group_leader(p))
			continue;

		task_lock(p);
		mm = p->mm;
		if (!mm) {
			/*
			 * total_vm and rss sizes do not exist for tasks with no
			 * mm so there's no need to report them; they can't be
			 * oom killed anyway.
			 */
			task_unlock(p);
			continue;
		}
		rss_sum += get_mm_rss(mm);
		task_unlock(p);
	} while_each_thread(g, p);

	return rss_sum;
}
#endif

void show_mem(unsigned int filter)
{
	pg_data_t *pgdat;
	unsigned long total = 0, reserved = 0, highmem = 0;
#if defined(CONFIG_LAB126)
	int free = 0, sharedp = 0, sharedp_mapped = 0;
	int shared = 0, cached = 0, slab = 0, i, shared_mapped = 0, user_pages =0;
	int user_cache =0, kernel_cache = 0;
#endif

	printk("Mem-Info:\n");
	show_free_areas(filter);

	for_each_online_pgdat(pgdat) {
		unsigned long flags;
		int zoneid;

		pgdat_resize_lock(pgdat, &flags);
		for (zoneid = 0; zoneid < MAX_NR_ZONES; zoneid++) {
			struct zone *zone = &pgdat->node_zones[zoneid];
			if (!populated_zone(zone))
				continue;

			total += zone->present_pages;
			reserved += zone->present_pages - zone->managed_pages;

			if (is_highmem_idx(zoneid))
				highmem += zone->present_pages;

#if defined(CONFIG_LAB126)
			struct page *page;
			unsigned long pfn = zone->zone_start_pfn, block_end_pfn;
			unsigned long end_pfn = pfn + zone->spanned_pages;

			/* Scan block by block. First and last block may be incomplete */
			pfn = zone->zone_start_pfn;

			/*
			 * Walk the zone in pageblock_nr_pages steps. If a page block spans
			 * a zone boundary, it will be double counted between zones. This does
			 * not matter as the mixed block count will still be correct
			 */
			for (; pfn < end_pfn; ) {
				if (!pfn_valid(pfn)) {
					pfn = ALIGN(pfn + 1, MAX_ORDER_NR_PAGES);
					continue;
				}

				block_end_pfn = ALIGN(pfn + 1, pageblock_nr_pages);
				block_end_pfn = min(block_end_pfn, end_pfn);

				for (; pfn < block_end_pfn; pfn++) {
					if (!pfn_valid_within(pfn))
						continue;

					page = pfn_to_page(pfn);

					if (PageReserved(page))
						continue;
					else if (PageSwapCache(page))
						cached++;
					else if (PageSlab(page))
						slab++;
					else if (!page_count(page))
						free++;
					else {
						shared += page_count(page) - 1;
						if (page_count(page) > 1)
							sharedp++;
						if (page_mapped(page)) {
							if (page_mapcount(page) > 1)
								sharedp_mapped++;
							shared_mapped += page_mapcount(page) - 1;
							user_pages++;
							if (page_mapping(page))
								user_cache++;
						} else if (page_mapping(page))
							kernel_cache++;
					}
				}
			}
#endif
		}
		pgdat_resize_unlock(pgdat, &flags);
	}

	printk("%lu pages RAM\n", total);
	printk("%lu pages HighMem/MovableOnly\n", highmem);
#ifdef CONFIG_CMA
	printk("%lu pages reserved\n", (reserved - totalcma_pages));
	printk("%lu pages cma reserved\n", totalcma_pages);
#else
	printk("%lu pages reserved\n", reserved);
#endif
#ifdef CONFIG_QUICKLIST
	printk("%lu pages in pagetable cache\n",
		quicklist_total_size());
#endif
#ifdef CONFIG_MEMORY_FAILURE
	printk("%lu pages hwpoisoned\n", atomic_long_read(&num_poisoned_pages));
#endif

#if defined(CONFIG_LAB126)
	printk("%d free pages\n", free);
	printk("%d slab pages\n", slab);
	printk("%d shared page count\n", shared);
	printk("%d shared pages\n", sharedp);
	printk("%d mapped shared page count\n", shared_mapped);
	printk("%d mapped shared pages\n", sharedp_mapped);
	printk("%d pages swap cached\n", cached);
	// printk("%d dma reserved pages\n",dma_reserved_count);
	printk("%d total user pages\n",user_pages);
	printk("%d RSS sum by tasks\n",rss_sum_by_tasks());
	printk("%d RSS sum by page stats\n", user_pages+shared_mapped);
	printk("%d user cache pages\n", user_cache);
	printk("%d kernel cache pages\n", kernel_cache);
#endif
}