/*
 * Copyright (c) 2015-2016, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Laxman Dewangan<ldewangan@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 */

#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/proc_fs.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include "of_private.h"

enum plugin_manager_match_type {
	PLUGIN_MANAGER_MATCH_EXACT,
	PLUGIN_MANAGER_MATCH_PARTIAL,
	PLUGIN_MANAGER_MATCH_GE,
	PLUGIN_MANAGER_MATCH_LT,
};

static int handle_connection_overrides_node(struct device_node *connector,
					    struct device_node *oride_parent,
					    struct device_node *target,
					    phandle *new_ph, phandle *old_ph,
					    int num_ph);

static struct property *__of_copy_property(const struct property *prop,
					   void *new_value, int val_len,
					   gfp_t flags)
{
	struct property *propn;
	int nlen;
	void *nval;

	propn = kzalloc(sizeof(*prop), flags);
	if (!propn)
		return NULL;

	propn->name = kstrdup(prop->name, flags);
	if (!propn->name)
		goto err_fail_name;

	nlen = (new_value) ? val_len : prop->length;
	nval = (new_value) ? new_value : prop->value;
	if (nlen > 0) {
		propn->value = kmalloc(nlen, flags);
		if (!propn->value)
			goto err_fail_value;
		memcpy(propn->value, nval, nlen);
		propn->length = nlen;
	}
	return propn;

err_fail_value:
	kfree(propn->name);
err_fail_name:
	kfree(propn);
	return NULL;
}

static void free_property(struct property *pp)
{
	if (!pp)
		return;

	kfree(pp->name);
	kfree(pp->value);
	kfree(pp);
}

static struct property *__of_string_append(struct device_node *target,
					   struct property *prop)
{
	struct property *new_prop, *tprop;
	const char *tprop_name, *curr_str;
	int slen, tlen, lenp;

	tprop_name = of_prop_next_string(prop, NULL);
	if (!tprop_name)
		return NULL;

	new_prop = kzalloc(sizeof(*new_prop), GFP_KERNEL);
	if (!new_prop)
		return NULL;

	new_prop->name = kstrdup(tprop_name, GFP_KERNEL);
	if (!new_prop->name)
		goto err_fail_name;

	curr_str = of_prop_next_string(prop, tprop_name);
	for (slen = 0; curr_str; curr_str = of_prop_next_string(prop, curr_str))
		slen += strlen(curr_str);

	tprop = of_find_property(target, tprop_name, &lenp);
	tlen = (tprop) ? tprop->length : 0;

	new_prop->value = kmalloc(slen + tlen, GFP_KERNEL);
	if (!new_prop->value)
		goto err_fail_value;

	if (tlen)
		memcpy(new_prop->value, tprop->value, tlen);

	if (slen) {
		curr_str = of_prop_next_string(prop, tprop_name);
		memcpy(new_prop->value + tlen, curr_str, slen);
	}

	new_prop->length = slen + tlen;

	return new_prop;

err_fail_value:
	kfree(new_prop->name);
err_fail_name:
	kfree(new_prop);

	return NULL;
}

struct device_node *of_get_child_by_addressed_name(
		const struct device_node *node, const char *name)
{
	struct device_node *child;
	const char *address_node_name;

	for_each_child_of_node(node, child) {
		address_node_name =  strrchr(child->full_name, '/');
		if (!strcmp(name, address_node_name))
			return child;
	}

	return NULL;
}

static int do_property_override_from_overlay(struct device_node *target,
					     struct device_node *overlay)
{
	struct property *prop;
	struct property *tprop;
	struct property *new_prop;
	const char *pval;
	int lenp = 0;
	int ret;

	pr_debug("Update properties from %s to %s\n", overlay->full_name,
		 target->full_name);

	for_each_property_of_node(overlay, prop) {
		/* Skip those we do not want to proceed */
		if (!strcmp(prop->name, "name") ||
			!strcmp(prop->name, "phandle") ||
			!strcmp(prop->name, "linux,phandle"))
				continue;
		if (!strcmp(prop->name, "delete-target-property")) {
			if (prop->length <= 0)
				continue;
			pval = (const char *)prop->value;
			pr_info("Removing Prop %s from target %s\n",
				pval, target->full_name);
			tprop = of_find_property(target, pval, &lenp);
			if (tprop)
				of_remove_property(target, tprop);
			continue;
		}

		if (!strcmp(prop->name, "append-string-property")) {
			if (prop->length <= 0)
				continue;

			new_prop = __of_string_append(target, prop);
			if (!new_prop) {
				pr_err("Prop %s can not be appended\n",
					of_prop_next_string(prop, NULL));
				return -EINVAL;
			}
			goto add_prop;
		}

		new_prop = __of_copy_property(prop, NULL, 0, GFP_KERNEL);
		if (!new_prop) {
			pr_err("Prop %s can not be duplicated\n",
				prop->name);
			return -EINVAL;
		}

add_prop:
		tprop = of_find_property(target, new_prop->name, &lenp);
		if (!tprop) {
			ret = of_add_property(target, new_prop);
			if (ret < 0) {
				pr_err("Prop %s can not be added on node %s\n",
					new_prop->name, target->full_name);
				goto cleanup;
			}
		} else {
			ret = of_update_property(target, new_prop);
			if (ret < 0) {
				pr_err("Prop %s can not be updated on node %s\n",
					new_prop->name, target->full_name);
				goto cleanup;
			}
		}
	}

	return 0;

cleanup:
	free_property(new_prop);
	return ret;
}

static int do_property_override_from_connector(struct device_node *connector,
					       struct device_node *target,
					       struct device_node *overlay)
{
	struct property *prop;
	struct property *tprop, *cprop;
	struct property *new_prop;
	int lenp = 0;
	int ret;

	for_each_property_of_node(overlay, prop) {
		/* Skip those we do not want to proceed */
		if (!strcmp(prop->name, "name") ||
			!strcmp(prop->name, "phandle") ||
			!strcmp(prop->name, "linux,phandle"))
				continue;

		cprop = of_find_property(connector, prop->value, &lenp);
		if (!cprop) {
			pr_err("Prop %s is not found in connector %s\n",
				prop->name, connector->full_name);
			continue;
		}

		new_prop = __of_copy_property(prop, cprop->value, cprop->length,
					      GFP_KERNEL);
		if (!new_prop) {
			pr_err("Prop %s can not be duplicated\n",
				prop->name);
			return -EINVAL;
		}

		tprop = of_find_property(target, new_prop->name, &lenp);
		if (!tprop) {
			ret = of_add_property(target, new_prop);
			if (ret < 0) {
				pr_err("Prop %s can not be added on node %s\n",
					new_prop->name, target->full_name);
				goto cleanup;
			}
		} else {
			ret = of_update_property(target, new_prop);
			if (ret < 0) {
				pr_err("Prop %s can not be updated on node %s\n",
					new_prop->name, target->full_name);
				goto cleanup;
			}
		}
	}

	return 0;

cleanup:
	free_property(new_prop);
	return ret;
}

static int plugin_manager_get_fabid(const char *id_str)
{
	int fabid = 0;
	int i;

	if (strlen(id_str) < 13)
		return -EINVAL;

	for (i = 0; i < 3; ++i) {
		if ((id_str[10 + i] >= '0') && (id_str[10 + i] <= '9'))
			fabid = fabid * 10 + id_str[10 + i] - '0';
		else
			return -EINVAL;
	}

	return fabid;
}

static bool plugin_manager_match_id(struct device_node *np, const char *id_name)
{
	struct property *prop;
	const char *in_str = id_name;
	int match_type = PLUGIN_MANAGER_MATCH_EXACT;
	int valid_str_len = strlen(id_name);
	int fabid = 0, prop_fabid;
	int i;

	if ((valid_str_len > 2) && (in_str[0] == '>') && (in_str[1] == '=')) {
		in_str += 2;
		valid_str_len -= 2;
		match_type = PLUGIN_MANAGER_MATCH_GE;
		goto match_type_done;
	}

	if ((valid_str_len > 1) && (in_str[0] == '<')) {
		in_str += 1;
		valid_str_len -= 1;
		match_type = PLUGIN_MANAGER_MATCH_LT;
		goto match_type_done;
	}

	if ((valid_str_len > 1) && (in_str[0] == '^')) {
		in_str += 1;
		valid_str_len -= 1;
		match_type = PLUGIN_MANAGER_MATCH_PARTIAL;
		goto match_type_done;
	}

	for (i = 0; i < valid_str_len; ++i) {
		if (in_str[i] == '*') {
			valid_str_len = i;
			match_type = PLUGIN_MANAGER_MATCH_PARTIAL;
			break;
		}
	}

match_type_done:
	if ((match_type == PLUGIN_MANAGER_MATCH_GE) ||
		(match_type == PLUGIN_MANAGER_MATCH_LT)) {
		fabid = plugin_manager_get_fabid(in_str);
		if (fabid < 0)
			return false;
	}

	for_each_property_of_node(np, prop) {
		/* Skip those we do not want to proceed */
		if (!strcmp(prop->name, "name") ||
			!strcmp(prop->name, "phandle") ||
			!strcmp(prop->name, "linux,phandle"))
				continue;
		switch (match_type) {
		case PLUGIN_MANAGER_MATCH_EXACT:
			if (strlen(prop->name) != valid_str_len)
				break;
			if (!memcmp(in_str, prop->name, valid_str_len))
				return true;
			break;

		case PLUGIN_MANAGER_MATCH_PARTIAL:
			if (strlen(prop->name) < valid_str_len)
				break;
			if (!memcmp(in_str, prop->name, valid_str_len))
				return true;
			break;

		case PLUGIN_MANAGER_MATCH_GE:
		case PLUGIN_MANAGER_MATCH_LT:
			if (strlen(prop->name) < 13)
				break;
			if (memcmp(in_str, prop->name, 10))
				break;
			prop_fabid = plugin_manager_get_fabid(prop->name);
			if (prop_fabid < 0)
				break;
			if (prop_fabid >= fabid &&
				match_type == PLUGIN_MANAGER_MATCH_GE)
				return true;
			if (prop_fabid < fabid &&
				match_type == PLUGIN_MANAGER_MATCH_LT)
				return true;
			break;
		default:
			break;
		}
	}

	return false;
}

static int do_property_overrides(struct device_node *target,
				 struct device_node *overlay)
{
	struct device_node *tchild, *ochild;
	const char *address_name;
	int ret;

	ret = do_property_override_from_overlay(target, overlay);
	if (ret < 0) {
		pr_err("Target %s update with overlay %s failed: %d\n",
			target->name, overlay->name, ret);
		return ret;
	}

	for_each_child_of_node(overlay, ochild) {
		address_name = strrchr(ochild->full_name, '/');
		tchild = of_get_child_by_addressed_name(target, address_name);
		if (!tchild) {
			pr_err("Overlay node %s not found in target node %s\n",
				ochild->full_name, target->full_name);
			continue;
		}
		ret = do_property_overrides(tchild, ochild);
		if (ret < 0) {
			pr_err("Target %s update with overlay %s failed: %d\n",
				tchild->name, ochild->name, ret);
			return ret;
		}
	}
	return 0;
}

static int do_property_copying(struct device_node *connector,
			       struct device_node *target,
			       struct device_node *overlay)
{
	struct device_node *tchild, *ochild;
	const char *address_name;
	int ret;

	ret = do_property_override_from_connector(connector, target, overlay);
	if (ret < 0) {
		pr_err("Target %s update with overlay %s failed: %d\n",
			target->name, overlay->name, ret);
		return ret;
	}

	for_each_child_of_node(overlay, ochild) {
		address_name = strrchr(ochild->full_name, '/');
		tchild = of_get_child_by_addressed_name(target, address_name);
		if (!tchild) {
			pr_err("Overlay node %s not found in target node %s\n",
				ochild->full_name, target->full_name);
			continue;
		}
		ret = do_property_copying(connector, tchild, ochild);
		if (ret < 0) {
			pr_err("Target %s update with overlay %s failed: %d\n",
				tchild->name, ochild->name, ret);
			return ret;
		}
	}

	return 0;
}

static int handle_properties_overrides(struct device_node *np,
				       struct device_node *target)
{
	struct device_node *overlay;
	int ret;

	if (!target) {
		target = of_parse_phandle(np, "target", 0);
		if (!target) {
			pr_err("Node %s does not have targer node\n",
				np->name);
			return -EINVAL;
		}
	}

	overlay = of_get_child_by_name(np, "_overlay_");
	if (!overlay) {
		pr_err("Node %s does not have Overlay\n", np->name);
		return -EINVAL;
	}

	ret = do_property_overrides(target, overlay);
	if (ret < 0) {
		pr_err("Target %s update with overlay %s failed: %d\n",
			target->name, overlay->name, ret);
		return -EINVAL;
	}

	return 0;
}

static int handle_properties_copying(struct device_node *connector,
				     struct device_node *np,
				     struct device_node *target)
{
	struct device_node *overlay;
	int ret;

	if (!target) {
		target = of_parse_phandle(np, "target", 0);
		if (!target) {
			pr_err("Node %s does not have targer node\n",
			       np->full_name);
			return -EINVAL;
		}
	}

	overlay = of_get_child_by_name(np, "_overlay_");
	if (!overlay) {
		pr_err("Node %s does not have Overlay\n",
		       np->full_name);
		return -EINVAL;
	}

	ret = do_property_copying(connector, target, overlay);
	if (ret < 0) {
		 pr_err("Target %s update with overlay %s failed: %d\n",
			target->full_name, overlay->full_name, ret);
		return -EINVAL;
	}

	return 0;
}

static int do_resolve_client_phandle(struct device_node *np, phandle *new_ph,
				     phandle *old_ph, int num_phandle)
{
	struct device_node *user;
	struct property *prop, *pp;
	phandle cur_phandle;
	const char *s;
	u32 pval;
	int index = 0, i;
	int lenp;
	int ret;

	of_property_for_each_string(np, "node-user-prop-names", prop, s) {
		ret = of_property_read_u32_index(np, "node-user",
						 index, &pval);
		if (ret < 0) {
			pr_err("Failed to read node-user from node %s\n",
				np->full_name);
			return ret;
		}

		user = of_find_node_by_phandle(pval);
		if (!user) {
			pr_err("Failed to get user listed in node %s\n",
				np->full_name);
			return -EINVAL;
		}

		pp = of_find_property(user, s, &lenp);
		if (!pp) {
			pr_err("Failed to get property from user node %s\n",
				user->full_name);
			return -EINVAL;
		}

		cur_phandle = be32_to_cpup(pp->value);
		for (i = 0; i < num_phandle; ++i) {
			if (old_ph[i] == cur_phandle)
				break;
		}

		if (i == num_phandle) {
			pr_err("User:%d %s does not linked with target node %s\n",
				index, user->full_name, np->full_name);
			return -EINVAL;
		}

		*(uint32_t *)pp->value = cpu_to_be32(new_ph[i]);

		index++;
	}

	return 0;
}

struct device_node *duplicate_single_node(struct device_node *np,
				          const char *path,
					  const char *new_name)
{
	struct device_node *dup;
	struct property *pp, *new_pp;
	int ret;
	const char *add_name;

	dup = kzalloc(sizeof(*dup), GFP_KERNEL);
	if (!dup)
		return NULL;

	if (new_name) {
		add_name = new_name;
	} else {
		add_name = strrchr(np->full_name, '/');
		add_name++;
	}
	dup->full_name = kasprintf(GFP_KERNEL, "%s/%s", path, add_name);

	of_node_init(dup);

	for_each_property_of_node(np, pp) {
		if (!strcmp(pp->name, "name"))
			new_pp = __of_copy_property(pp, (void *)add_name,
						    strlen(add_name),
						    GFP_KERNEL);
		else
			new_pp = __of_copy_property(pp, NULL, 0, GFP_KERNEL);
		if (!new_pp)
			return NULL;

		ret = of_add_property(dup, new_pp);
		if (ret < 0) {
			pr_err("Prop %s can not be added on node %s\n",
				new_pp->name, dup->full_name);
			free_property(new_pp);
			return NULL;
		}
	}

	dup->name = __of_get_property(dup, "name", NULL) ? : "<NULL>";
	dup->type = __of_get_property(dup, "device_type", NULL) ? : "<NULL>";

	return dup;
}

struct device_node *get_copy_of_node(struct device_node *np,
				     const char *path, const char *new_name)
{
	struct device_node *dup;
	struct device_node *child, *child_dup;
	struct device_node *prev_child = NULL;

	dup = duplicate_single_node(np, path, new_name);
	if (!dup)
		return NULL;

	for_each_child_of_node(np, child) {
		child_dup = get_copy_of_node(child, dup->full_name, NULL);
		if (!child_dup)
			return NULL;
		child_dup->parent = dup;
		child_dup->sibling = NULL;
		if (!prev_child)
			dup->child = child_dup;
		else
			prev_child->sibling = child_dup;
		prev_child = child_dup;
	}

	return dup;
}

static int of_get_next_phandle(void)
{
	static phandle curr_handle = 0;
	static bool first_time = true;
	struct device_node *np;
	phandle next_handle;
	unsigned long flags;

	raw_spin_lock_irqsave(&devtree_lock, flags);

	if (first_time) {
		for_each_of_allnodes(np) {
			if (np->phandle > curr_handle)
				curr_handle = np->phandle;
		}
		first_time = false;
	}

	next_handle = curr_handle++;
	raw_spin_unlock_irqrestore(&devtree_lock, flags);

	return next_handle;
}

static int adjust_phandle_reference(struct device_node *np, phandle *new_phlist,
				    phandle *old_phlist, int curr_index)
{
	struct device_node *child;
	struct property *ph_prop, *lph_prop;
	int lenp;

	ph_prop =  of_find_property(np, "phandle", &lenp);
	lph_prop = of_find_property(np, "linux,phandle", &lenp);
	if (ph_prop || lph_prop) {
		np->phandle = of_get_next_phandle();

		new_phlist[curr_index] = np->phandle;
		old_phlist[curr_index++] = (ph_prop) ? be32_to_cpup(ph_prop->value) :
							be32_to_cpup(lph_prop->value);
		if (ph_prop->length >= 4)
			*(uint32_t *)ph_prop->value = cpu_to_be32(np->phandle);
		if (lph_prop->length >= 4)
			*(uint32_t *)lph_prop->value = cpu_to_be32(np->phandle);

	}

	for_each_child_of_node(np, child)
		curr_index = adjust_phandle_reference(child, new_phlist,
						      old_phlist, curr_index);

	return curr_index;
}

static int _attach_node_and_children(struct device_node *np)
{
	struct device_node *child;

	__of_attach_node_sysfs(np);
	for_each_child_of_node(np, child)
		_attach_node_and_children(child);

	return 0;
}

static int handle_connection_node_copy(struct device_node *connector,
				       struct device_node *oride)
{
	struct device_node *dup, *src;
	struct device_node *bus;
	struct device_node *last_sibling;
	phandle num_phandle;
	const char *node_name = NULL;
	phandle new_ph_list[100], old_ph_list[100];
	unsigned long flags;

	bus = of_parse_phandle(connector, "bus", 0);
	if (!bus) {
		pr_err("Node %s does not have bus property\n", connector->full_name);
		return -EINVAL;
	}

	src = of_parse_phandle(oride, "source", 0);
	if (!src) {
		pr_err("Node %s does not have source node\n", oride->full_name);
		return -EINVAL;
	}

	if (of_property_read_string(oride, "node-name", &node_name))
		node_name = NULL;

	dup = get_copy_of_node(src, bus->full_name, node_name);
	if (!dup) {
		pr_err("Failed to create duplicate of node %s\n", src->full_name);
		return -EINVAL;
	}

	num_phandle = adjust_phandle_reference(dup, new_ph_list,
					       old_ph_list, 0);

	mutex_lock(&of_mutex);
	raw_spin_lock_irqsave(&devtree_lock, flags);

	/* Add target node as the lasy child of bus node */
	if (!bus->child) {
		bus->child = dup;
	} else {
		last_sibling = bus->child;
		while(!last_sibling->sibling)
			last_sibling = last_sibling->sibling;
		last_sibling->sibling = dup;
	}
	dup->sibling = NULL;
	dup->parent = bus;

	raw_spin_unlock_irqrestore(&devtree_lock, flags);
	_attach_node_and_children(dup);

	mutex_unlock(&of_mutex);

	return handle_connection_overrides_node(connector, oride, dup,
						new_ph_list, old_ph_list,
						num_phandle);
}

static int handle_connection_overrides_node(struct device_node *connector,
					    struct device_node *oride_parent,
					    struct device_node *target,
					    phandle *new_ph, phandle *old_ph,
					    int num_ph)
{
	struct device_node *child;
	int ret;

	for_each_available_child_of_node(oride_parent, child) {
		if (of_property_read_bool(child, "properties-override")) {
			ret = handle_properties_overrides(child, target);
		} else if (of_property_read_bool(child, "properties-copying")) {
			ret = handle_properties_copying(connector, child, target);
		} else if (of_property_read_bool(child, "resolve-client-phandle")) {
			ret = do_resolve_client_phandle(child, new_ph,
							old_ph, num_ph);
		} else if (of_property_read_bool(child, "node-copying")) {
			ret = handle_connection_node_copy(connector, child);
		} else {
			ret = handle_properties_overrides(child, target);
		}

		if (ret < 0) {
			pr_err("Failed to apply overrides at node %s: %d\n",
				child->full_name, ret);
			return ret;
		}
	}

	return 0;
}

static int lookup_for_connectors(struct device_node *connector_list,
				 struct device_node *ids,
				 struct device_node *connection,
				 const char *module_name)
{
	struct device_node *child;
	struct device_node *connector;
	bool found;
	int ret = 0;

	found = plugin_manager_match_id(ids, module_name);
	if (!found)
		goto look_for_child;

	pr_info("node %s match with board %s\n", ids->full_name, module_name);

	connector = of_get_child_by_name(connector_list, ids->name);
	if (!connector) {
		pr_info("Connector node %s is not found\n", ids->name);
		return -EINVAL;
	}

	ret = handle_connection_overrides_node(connector, connection, NULL,
					       NULL, NULL, 0);
	if (ret < 0) {
		pr_info("Failed to handle connection %s with connector %s: %d\n",
			connection->full_name, connector->name, ret);
		return ret;
	}

look_for_child:
	for_each_available_child_of_node(ids, child) {
		ret = lookup_for_connectors(connector_list, child,
					    connection, module_name);
		if (ret < 0)
			return ret;
	}

	return ret;
}

static int __init connection_manager(struct device_node *np_frag)
{
	struct device_node *ids;
	struct device_node *connector_list;
	struct device_node *connection;
	struct property *prop;
	const char *bname;
	int ret;

	connector_list = of_parse_phandle(np_frag, "connectors", 0);
	if (!connector_list) {
		pr_err("Failed to get the conntors node\n");
		return -EINVAL;
	}

	ids = of_find_node_by_path("/chosen/plugin-manager/ids");
	if (!ids) {
		pr_err("chosen/plugin-manager does'nt have ids\n");
		return -EINVAL;
	}

	for_each_available_child_of_node(np_frag, connection) {
		of_property_for_each_string(connection, "ids", prop, bname) {
			ret = lookup_for_connectors(connector_list, ids,
						    connection, bname);
			if (ret < 0) {
				pr_err("Failed to handle connection %s: %d\n",
					connection->full_name, ret);
				return ret;
			}
		}
	}

	return 0;
}

static int __init plugin_manager(struct device_node *np)
{
	struct device_node *board_np, *nct_np, *odm_np, *cnp;
	struct device_node *config_np, *chip_np;
	const char *bname;
	struct property *prop;
	int board_count;
	int odm_count, nct_count, chip_id_count;
	int cname_count, cval_count;
	int nchild;
	bool found = false;
	bool override_on_all_match;
	int ret;

	override_on_all_match = of_property_read_bool(np,
					"enable-override-on-all-matches");

	cname_count = of_property_count_strings(np, "config-names");
	cval_count = of_property_count_u32_elems(np, "configs");
	if (cname_count != cval_count) {
		pr_err("Node %s does not have config-names and configs\n",
			np->name);
		return -EINVAL;
	}

	board_count = of_property_count_strings(np, "ids");
	odm_count = of_property_count_strings(np, "odm-data");
	nct_count = of_property_count_strings(np, "nct-data");
	chip_id_count = of_property_count_strings(np, "chip-id");
	if ((board_count <= 0) && (odm_count <= 0) && (cname_count <= 0) &&
	    (nct_count <= 0) && (chip_id_count <= 0)) {
		pr_err("Node %s does not have property ids, nct and odm data\n",
			np->name);
		return -EINVAL;
	}

	nchild = of_get_child_count(np);
	if (!nchild) {
		pr_err("Node %s does not have Overlay child\n", np->name);
		return -EINVAL;
	}

	/* Match the IDs or odm data */
	board_np = of_find_node_by_path("/chosen/plugin-manager/ids");
	odm_np = of_find_node_by_path("/chosen/plugin-manager/odm-data");
	nct_np = of_find_node_by_path("/chosen/plugin-manager/nct-data");
	chip_np = of_find_node_by_path("/chosen/plugin-manager/chip-id");
	config_np = of_find_node_by_path("/chosen/plugin-manager/configs");
	if (!board_np && !odm_np && !config_np && !nct_np && !chip_np) {
		pr_err("chosen/plugin-manager does'nt have ids, nct and odm-data\n");
		return -EINVAL;
	}

	if ((board_count > 0) && board_np) {
		of_property_for_each_string(np, "ids", prop, bname) {
			found = plugin_manager_match_id(board_np, bname);
			if (found) {
				pr_info("node %s match with board %s\n",
					np->full_name, bname);
				if (override_on_all_match)
					break;
				goto search_done;
			}
		}

		if (override_on_all_match && !found)
			return 0;
	}

	if ((odm_count > 0) && odm_np) {
		of_property_for_each_string(np, "odm-data", prop, bname) {
			found = of_property_read_bool(odm_np, bname);
			if (found) {
				pr_info("node %s match with odm-data %s\n",
					np->full_name, bname);
				if (override_on_all_match)
					break;
				goto search_done;
			}
		}

		if (override_on_all_match && !found)
			return 0;
	}

	if ((nct_count > 0) && nct_np) {
		of_property_for_each_string(np, "nct-data", prop, bname) {
			found = of_property_read_bool(nct_np, bname);
			if (found) {
				pr_info("node %s match with nct-data %s\n",
					np->full_name, bname);
				if (override_on_all_match)
					break;
				goto search_done;
			}
		}

		if (override_on_all_match && !found)
			return 0;
	}

	if ((chip_id_count > 0) && chip_np) {
		of_property_for_each_string(np, "chip-id", prop, bname) {
			found = of_property_read_bool(chip_np, bname);
			if (found) {
				pr_info("node %s match with chip-id %s\n",
					np->full_name, bname);
				if (override_on_all_match)
					break;
				goto search_done;
			}
		}

		if (override_on_all_match && !found)
			return 0;
	}

	if ((cname_count > 0) && config_np) {
		int index = 0;
		u32 pval = 0, pmv = 0, mask, value;

		of_property_for_each_string(np, "config-names", prop, bname) {
			ret = of_property_read_u32_index(np, "configs",
					index, &pmv);
			if (ret < 0) {
				pr_info("node %s do not have proper configs\n",
					np->name);
				return ret;
			}
			index++;
			ret = of_property_read_u32(config_np, bname, &pval);
			if (ret < 0)
				continue;

			mask = (pmv >> 8) & 0xFF;
			value = pmv & 0xFF;
			pval &= 0xFF;
			found = ((pval & mask) == value);
			if (found) {
				pr_info("node %s match with config %s\n",
					np->full_name, bname);
				if (override_on_all_match)
					break;
				goto search_done;
			}
		}

		if (override_on_all_match && !found)
			return 0;
	}

search_done:
	if (!found)
		return 0;

	for_each_child_of_node(np, cnp)
		handle_properties_overrides(cnp, NULL);

	return 0;
}

static int __init plugin_manager_init(void)
{
	struct device_node *pm_node;
	struct device_node *child;
	int ret;

	pr_info("Initializing plugin-manager\n");

	pm_node = of_find_node_by_path("/plugin-manager");
	if (!pm_node) {
		pr_info("Plugin-manager not available\n");
		return 0;
	}

	if (!of_device_is_available(pm_node)) {
		pr_info("Plugin-manager status disabled\n");
		return 0;
	}

	for_each_available_child_of_node(pm_node, child) {
		bool connection;

		connection = of_property_read_bool(child, "connection-type");
		if (connection)
			ret = connection_manager(child);
		else
			ret = plugin_manager(child);

		if (ret < 0)
			pr_err("Error in parsing node %s: %d\n",
				child->full_name, ret);
	}
	return 0;
}
core_initcall(plugin_manager_init);
