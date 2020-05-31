/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#ifndef _UAPI_LINUX_FALCON_STORAGE_H
#define _UAPI_LINUX_FALCON_STORAGE_H

struct falcon_blk_sbios_call_container {
	void *ptr;
	int size;
};

#define F_SBIOS_CALL	_IOWR(0xee, 1, struct falcon_blk_sbios_call_container)

#endif
