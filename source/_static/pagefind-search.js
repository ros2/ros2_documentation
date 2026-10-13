// Pagefind search UI for the docs sidebar modal and search.html.
// Facet labels, the current distro, and bundle paths come from #ros-pagefind-config.

const PAGEFIND_AUTO_META = new Set(['title', 'image', 'image_alt', 'url']);

function readConfig() {
  const el = document.getElementById('ros-pagefind-config');
  if (!el) {
    return null;
  }
  try {
    const parsed = JSON.parse(el.textContent || '');
    return parsed && typeof parsed === 'object' ? parsed : null;
  } catch {
    return null;
  }
}

const config = readConfig();
const resultMetaFields = Array.isArray(config?.resultMetaFields) ? config.resultMetaFields : [];
const alwaysShowFilterKeys = (Array.isArray(config?.alwaysShowFilterKeys) ? config.alwaysShowFilterKeys : [])
  .map((key) => String(key || '').trim())
  .filter(Boolean);
const currentDistro = String(config?.currentDistro || '');
const searchResultsUrl = String(config?.searchResultsUrl || 'search.html');
const bundlePrefix = String(config?.bundlePrefix || './pagefind/');
const componentJs = String(config?.componentJs || '');
const showBreadcrumb = Boolean(config?.showBreadcrumb);

function getPagefindInstance() {
  return window.PagefindComponents?.getInstanceManager?.()?.getInstance?.('default') ?? null;
}

function normalizeSearchFilters(raw) {
  const out = {};
  if (!raw || typeof raw !== 'object') {
    return out;
  }
  for (const field of resultMetaFields) {
    const key = field.key;
    if (!key) {
      continue;
    }
    let values = raw[key];
    if (values == null) {
      const match = Object.keys(raw).find((name) => name.toLowerCase() === key.toLowerCase());
      values = match != null ? raw[match] : null;
    }
    const arr = Array.isArray(values)
      ? values
      : (values != null && values !== '' ? [values] : []);
    const cleaned = arr.map((value) => String(value).trim()).filter(Boolean);
    if (cleaned.length) {
      out[key] = cleaned;
    }
  }
  return out;
}

function appendFiltersToParams(params, filters) {
  for (const field of resultMetaFields) {
    const key = field.key;
    const values = filters[key];
    if (!values?.length) {
      continue;
    }
    values.forEach((value) => {
      const trimmed = String(value).trim();
      if (trimmed) {
        params.append(key, trimmed);
      }
    });
  }
}

function parseFiltersFromUrl(searchString) {
  const params = new URLSearchParams(searchString ?? window.location.search);
  const filters = {};
  for (const field of resultMetaFields) {
    const key = field.key;
    if (!key) {
      continue;
    }
    const values = params.getAll(key).map((value) => value.trim()).filter(Boolean);
    if (values.length) {
      filters[key] = values;
    }
  }
  return filters;
}

function urlHasFacetParams(searchString) {
  const params = new URLSearchParams(searchString ?? window.location.search);
  return resultMetaFields.some((field) => {
    const key = field.key;
    return key && params.getAll(key).some((value) => value.trim());
  });
}

function buildSearchResultsHref(term, searchFilters) {
  const params = new URLSearchParams();
  const query = (term || '').trim();
  if (query) {
    params.set('q', query);
  }
  appendFiltersToParams(params, normalizeSearchFilters(searchFilters));
  const encoded = params.toString();
  return encoded ? `${searchResultsUrl}?${encoded}` : searchResultsUrl;
}

function getModalSearchTerm(event) {
  const instance = getPagefindInstance();
  const fromInstance = (instance?.searchTerm || '').trim();
  if (fromInstance) {
    return fromInstance;
  }
  const pathInput = event.composedPath().find((node) => node instanceof HTMLInputElement);
  return (pathInput?.value || '').trim();
}

function bindModalEnterToSearchPage() {
  document.addEventListener(
    'keydown',
    (event) => {
      if (event.key !== 'Enter') {
        return;
      }
      const modal = document.querySelector('pagefind-modal');
      if (!modal) {
        return;
      }
      const inModal = event.composedPath().some(
        (node) => node instanceof Element && (node === modal || modal.contains(node)),
      );
      if (!inModal) {
        return;
      }
      const query = getModalSearchTerm(event);
      if (!query) {
        return;
      }
      event.preventDefault();
      event.stopImmediatePropagation();
      const filters = mergeDefaultDistributionFilter(
        normalizeSearchFilters(getPagefindInstance()?.searchFilters),
      );
      window.location.href = buildSearchResultsHref(query, filters);
    },
    true,
  );
}

function formatMetaValue(raw) {
  if (raw == null) {
    return '';
  }
  if (Array.isArray(raw)) {
    return raw.map((value) => String(value).trim()).filter(Boolean).join(', ');
  }
  return String(raw).trim();
}

function labelForMetaKey(key) {
  const field = resultMetaFields.find((item) => item.key === key);
  return (field?.label || '').trim() || String(key || '');
}

function lookupNamed(obj, key) {
  if (!obj || typeof obj !== 'object' || !key) {
    return undefined;
  }
  if (Object.prototype.hasOwnProperty.call(obj, key)) {
    return obj[key];
  }
  const lower = String(key).toLowerCase();
  const found = Object.keys(obj).find((name) => name.toLowerCase() === lower);
  return found == null ? undefined : obj[found];
}

function valuesForField(data, key) {
  const filters = lookupNamed(data?.filters, key);
  if (Array.isArray(filters)) {
    return filters.map((value) => String(value).trim()).filter(Boolean);
  }
  if (filters && typeof filters === 'object' && Object.keys(filters).length) {
    return Object.keys(filters).map((value) => String(value).trim()).filter(Boolean);
  }
  const meta = lookupNamed(data?.meta, key);
  if (Array.isArray(meta)) {
    return meta.map((value) => String(value).trim()).filter(Boolean);
  }
  if (meta == null) {
    return [];
  }
  const text = String(meta).trim();
  return text ? [text] : [];
}

function entriesFromRecord(data) {
  const entries = [];
  resultMetaFields.forEach((field) => {
    const key = field.key;
    if (!key || PAGEFIND_AUTO_META.has(key)) {
      return;
    }
    const value = formatMetaValue(valuesForField(data, key));
    if (!value) {
      return;
    }
    entries.push({ key, value });
  });
  return entries;
}

function breadcrumbFromRecord(data) {
  if (!showBreadcrumb) {
    return '';
  }
  const raw = lookupNamed(data?.meta, 'breadcrumb');
  if (raw == null) {
    return '';
  }
  if (Array.isArray(raw)) {
    return raw.map((value) => String(value).trim()).filter(Boolean).join(' › ');
  }
  return String(raw).trim();
}

function resultPathKeys(href) {
  const keys = new Set();
  const add = (url) => {
    const path = url.pathname
      .replace(/\/index\.html$/i, '')
      .replace(/\.html$/i, '')
      .replace(/\/+$/, '')
      .toLowerCase();
    if (path) {
      keys.add(path);
    }
  };
  try {
    add(new URL(href || '', document.baseURI));
  } catch {
    // Ignore unparsable result URLs.
  }
  try {
    add(new URL(href || '', new URL('/', document.baseURI)));
  } catch {
    // Ignore unparsable site-root URLs.
  }
  return keys;
}

function buildMetadataBlock(entries) {
  const block = document.createElement('p');
  block.className = 'pf-result-meta-block ros-page-meta-summary';
  block.dataset.rosMetaBuilt = '1';
  entries.forEach(({ key, value }, index) => {
    if (index > 0) {
      block.appendChild(document.createTextNode(' | '));
    }
    const nameEl = document.createElement('strong');
    const label = labelForMetaKey(key);
    nameEl.textContent = label.endsWith(':') ? label : `${label}:`;
    block.appendChild(nameEl);
    block.appendChild(document.createTextNode(` ${formatMetaValue(value)}`));
  });
  return block;
}

function buildBreadcrumbEl(text) {
  const el = document.createElement('p');
  el.className = 'pf-result-breadcrumb';
  el.dataset.rosBreadcrumbBuilt = '1';
  const label = document.createElement('span');
  label.className = 'pf-result-breadcrumb-label';
  label.textContent = 'In';
  el.appendChild(label);
  el.appendChild(document.createTextNode(` ${text}`));
  return el;
}

function clearResultChrome(resultItem) {
  resultItem.querySelectorAll('.pf-result-meta-block, .pf-result-breadcrumb').forEach((el) => {
    el.remove();
  });
}

function insertResultChrome(content, { breadcrumb, entries }) {
  content.querySelectorAll('.pf-result-meta-block, .pf-result-breadcrumb').forEach((el) => {
    el.remove();
  });
  const nodes = [];
  if (breadcrumb) {
    nodes.push(buildBreadcrumbEl(breadcrumb));
  }
  if (entries?.length) {
    nodes.push(buildMetadataBlock(entries));
  }
  if (!nodes.length) {
    return;
  }
  const title = content.querySelector('.pf-result-title');
  if (title && title.parentNode === content) {
    let anchor = title;
    nodes.forEach((node) => {
      anchor.insertAdjacentElement('afterend', node);
      anchor = node;
    });
    return;
  }
  nodes.reverse().forEach((node) => {
    content.prepend(node);
  });
}

let resultMetaKey = '';
let resultHandles = [];
let resultMetaByPath = new Map();
let resultHandleCursor = 0;
let resultMetaChain = Promise.resolve();

function enqueueResultMeta(task) {
  const run = resultMetaChain.then(task, task);
  resultMetaChain = run.then(() => undefined, () => undefined);
  return run;
}

function currentResultMetaKey() {
  const instance = getPagefindInstance();
  const term = (instance?.searchTerm || '').trim();
  if (!term) {
    return '';
  }
  return `${term}\n${filterCountsKey(instance?.searchFilters)}`;
}

function resetResultMetaCache() {
  resultMetaKey = '';
  resultHandles = [];
  resultMetaByPath = new Map();
  resultHandleCursor = 0;
  collectAllPfResultLis().forEach((item) => {
    delete item.dataset.rosMetaAttempted;
    clearResultChrome(item);
  });
}

async function handlesForCurrentSearch() {
  const key = currentResultMetaKey();
  if (!key) {
    if (resultMetaKey) {
      resetResultMetaCache();
    }
    return [];
  }
  if (key === resultMetaKey) {
    return resultHandles;
  }
  const instance = getPagefindInstance();
  const pagefind = instance?.__pagefind__;
  if (!pagefind || typeof pagefind.search !== 'function') {
    return [];
  }
  const search = await pagefind.search((instance.searchTerm || '').trim(), {
    filters: normalizeSearchFilters(instance.searchFilters),
  });
  if (currentResultMetaKey() !== key) {
    return handlesForCurrentSearch();
  }
  resetResultMetaCache();
  resultMetaKey = key;
  resultHandles = search?.results || [];
  return resultHandles;
}

function cachedEntriesForHref(href) {
  for (const key of resultPathKeys(href)) {
    if (resultMetaByPath.has(key)) {
      return resultMetaByPath.get(key);
    }
  }
  return undefined;
}

function storeRecord(data) {
  const record = {
    entries: entriesFromRecord(data),
    breadcrumb: breadcrumbFromRecord(data),
  };
  resultPathKeys(data?.url).forEach((key) => {
    resultMetaByPath.set(key, record);
  });
}

async function ensureMetaForHrefs(hrefs) {
  await enqueueResultMeta(async () => {
    const pending = hrefs.filter((href) => cachedEntriesForHref(href) === undefined);
    if (!pending.length) {
      return;
    }
    const handles = await handlesForCurrentSearch();
    const wanted = new Set();
    pending.forEach((href) => {
      resultPathKeys(href).forEach((key) => wanted.add(key));
    });
    while (wanted.size && resultHandleCursor < handles.length) {
      const handle = handles[resultHandleCursor];
      resultHandleCursor += 1;
      if (!handle || typeof handle.data !== 'function') {
        continue;
      }
      let data;
      try {
        data = await handle.data();
      } catch {
        continue;
      }
      storeRecord(data);
      resultPathKeys(data?.url).forEach((key) => wanted.delete(key));
      pending.forEach((href) => {
        if (cachedEntriesForHref(href) !== undefined) {
          resultPathKeys(href).forEach((key) => wanted.delete(key));
        }
      });
    }
  });
}

const metadataDecorationInFlight = new WeakSet();

async function decorateResultItem(resultItem) {
  if (!(resultItem instanceof HTMLElement)) {
    return;
  }
  const link = resultItem.querySelector('.pf-result-link');
  const content = resultItem.querySelector('.pf-result-content');
  if (!link || !content) {
    return;
  }
  const existingBlock = resultItem.querySelector('.pf-result-meta-block');
  const existingCrumb = resultItem.querySelector('.pf-result-breadcrumb');
  const hasGoodBlock = existingBlock instanceof HTMLElement && existingBlock.dataset.rosMetaBuilt === '1';
  const hasGoodCrumb = existingCrumb instanceof HTMLElement && existingCrumb.dataset.rosBreadcrumbBuilt === '1';
  const hasChrome = hasGoodBlock || hasGoodCrumb;
  const skipEmptyRetry = resultItem.dataset.rosMetaAttempted === '1' && !existingBlock && !existingCrumb;
  if (hasChrome || skipEmptyRetry || metadataDecorationInFlight.has(resultItem)) {
    return;
  }
  metadataDecorationInFlight.add(resultItem);
  try {
    const href = link.getAttribute('href');
    await ensureMetaForHrefs([href]);
    const record = cachedEntriesForHref(href);
    if (record === undefined) {
      if (resultMetaKey && resultHandleCursor >= resultHandles.length) {
        resultItem.dataset.rosMetaAttempted = '1';
      }
      return;
    }
    const entries = record.entries || [];
    const breadcrumb = record.breadcrumb || '';
    if (entries.length || breadcrumb) {
      insertResultChrome(content, { breadcrumb, entries });
      delete resultItem.dataset.rosMetaAttempted;
    } else {
      resultItem.dataset.rosMetaAttempted = '1';
    }
  } finally {
    metadataDecorationInFlight.delete(resultItem);
  }
}

function filterKeyInTotalFilters(key, totalFilters) {
  if (!key || !totalFilters || typeof totalFilters !== 'object') {
    return false;
  }
  if (Object.prototype.hasOwnProperty.call(totalFilters, key)) {
    return true;
  }
  const lower = String(key).toLowerCase();
  return Object.keys(totalFilters).some((name) => name.toLowerCase() === lower);
}

function isConfiguredFacetKey(key) {
  const lower = String(key || '').toLowerCase();
  return resultMetaFields.some((field) => field.key && field.key.toLowerCase() === lower);
}

function resolveFilterKeyFromGroup(el, totalFilters) {
  if (!(el instanceof HTMLElement)) {
    return null;
  }
  if (el.dataset.rosPfFilterKey) {
    return el.dataset.rosPfFilterKey;
  }
  const attr =
    el.getAttribute('data-filter-name') ||
    el.dataset.filterName ||
    el.getAttribute('data-pf-filter') ||
    el.getAttribute('data-filter');
  if (attr) {
    return attr;
  }
  const summary = el.querySelector('summary, button');
  let text = '';
  if (summary) {
    for (const node of summary.childNodes) {
      if (node.nodeType === Node.TEXT_NODE) {
        text += node.textContent || '';
      }
    }
    text = text.trim() || (summary.textContent || '').trim();
  }
  if (!text) {
    return null;
  }
  const cleaned = text.replace(/\s*\d+\s*$/, '').trim();
  for (const field of resultMetaFields) {
    if (field.key && cleaned.toLowerCase() === field.key.toLowerCase()) {
      return field.key;
    }
    if (field.label && cleaned.toLowerCase() === field.label.toLowerCase()) {
      return field.key;
    }
  }
  for (const name of Object.keys(totalFilters || {})) {
    if (cleaned.toLowerCase() === name.toLowerCase()) {
      return name;
    }
  }
  return null;
}

function collectFilterGroups(root) {
  const details = [...root.querySelectorAll('details')];
  if (details.length) {
    return details;
  }
  const tagged = [
    ...root.querySelectorAll('[data-filter-name], [data-pf-filter], [data-filter]'),
  ];
  if (tagged.length) {
    return tagged;
  }
  return [...root.children].filter((node) => node instanceof HTMLElement);
}

function isAlwaysShowFilterKey(key) {
  const lower = String(key || '').toLowerCase();
  return alwaysShowFilterKeys.some((name) => name.toLowerCase() === lower);
}

function facetGroupTitle(key) {
  return String(key || '').trim().toUpperCase();
}

function filterPaneRoot(pane) {
  if (!(pane instanceof HTMLElement)) {
    return null;
  }
  return pane.querySelector('.pf-filter-pane') || pane;
}

function customizeFilterPane(pane, totalFilters) {
  if (!(pane instanceof HTMLElement)) {
    return;
  }
  const root = filterPaneRoot(pane);
  if (!root) {
    return;
  }
  const groups = collectFilterGroups(root);
  const matched = [];
  groups.forEach((group) => {
    const key = resolveFilterKeyFromGroup(group, totalFilters);
    const show = Boolean(key) && isConfiguredFacetKey(key);
    const nextDisplay = show ? '' : 'none';
    if (group.style.display !== nextDisplay) {
      group.style.display = nextDisplay;
    }
    if (!show) {
      return;
    }
    if (group.dataset.rosPfFilterKey !== key) {
      group.dataset.rosPfFilterKey = key;
    }
    matched.push({ key, el: group });
  });
  const container = matched[0]?.el?.parentElement;
  if (!container) {
    return;
  }
  const ordered = [];
  resultMetaFields.forEach((field) => {
    const entry = matched.find((item) => item.key.toLowerCase() === field.key.toLowerCase());
    if (entry) {
      ordered.push(entry.el);
    }
  });
  const alreadyOrdered = ordered.every((el, index) => container.children[index] === el);
  if (!alreadyOrdered) {
    ordered.forEach((el) => {
      container.appendChild(el);
    });
  }
}

function ensureAlwaysShowPlaceholders(scopeEl, totalFilters) {
  if (!(scopeEl instanceof Element)) {
    return;
  }
  const hasIndexedFilters =
    totalFilters &&
    typeof totalFilters === 'object' &&
    Object.keys(totalFilters).length > 0;

  scopeEl.querySelectorAll('.ros-search-facets').forEach((aside) => {
    if (!(aside instanceof HTMLElement)) {
      return;
    }
    aside.classList.toggle('ros-search-facets--placeholders', !hasIndexedFilters);
    aside.classList.toggle('ros-search-facets--native', hasIndexedFilters);
    aside.querySelectorAll('pagefind-filter-pane').forEach((pane) => {
      if (!(pane instanceof HTMLElement)) {
        return;
      }
      pane.hidden = !hasIndexedFilters;
      pane.style.display = hasIndexedFilters ? '' : 'none';
    });

    let host = aside.querySelector('.ros-facet-placeholders');
    if (!host) {
      host = document.createElement('div');
      host.className = 'ros-facet-placeholders';
      aside.appendChild(host);
    }

    const missingFields = resultMetaFields.filter((field) => {
      const key = field.key;
      return key && isAlwaysShowFilterKey(key) && !filterKeyInTotalFilters(key, totalFilters);
    });
    host.hidden = missingFields.length === 0;

    const wanted = new Set(missingFields.map((field) => field.key.toLowerCase()));
    [...host.querySelectorAll('[data-ros-facet-placeholder]')].forEach((el) => {
      const key = (el.getAttribute('data-ros-facet-placeholder') || '').toLowerCase();
      if (!wanted.has(key)) {
        el.remove();
      }
    });

    missingFields.forEach((field) => {
      const key = field.key;
      const title = facetGroupTitle(key);
      let details = host.querySelector(`[data-ros-facet-placeholder="${CSS.escape(key)}"]`);
      if (!details) {
        details = document.createElement('details');
        details.className = 'ros-facet-placeholder';
        details.setAttribute('data-ros-facet-placeholder', key);
        details.setAttribute('data-filter-name', key);
        details.open = true;
        const summary = document.createElement('summary');
        summary.textContent = title;
        const empty = document.createElement('p');
        empty.className = 'ros-facet-placeholder-empty';
        empty.textContent = 'No options available';
        details.appendChild(summary);
        details.appendChild(empty);
        host.appendChild(details);
      } else {
        const summary = details.querySelector('summary');
        if (summary && summary.textContent !== title) {
          summary.textContent = title;
        }
      }
    });

    const placeholderOrder = missingFields
      .map((field) => host.querySelector(`[data-ros-facet-placeholder="${CSS.escape(field.key)}"]`))
      .filter(Boolean);
    const placeholdersOrdered = placeholderOrder.every((el, index) => host.children[index] === el);
    if (!placeholdersOrdered) {
      placeholderOrder.forEach((el) => {
        host.appendChild(el);
      });
    }
  });
}

// Counts beside filters ignore the search words. They describe the pages that
// match the selected filters alone. The result summary still counts the list.
let filterOnlyCounts = null;
let filterOnlyCountsKey = '';
let filterOnlyRequest = 0;

function filterCountsKey(filters) {
  const normalized = normalizeSearchFilters(filters);
  return JSON.stringify(
    Object.keys(normalized)
      .sort()
      .map((key) => [key, [...normalized[key]].sort()]),
  );
}

function countInFilterMap(map, name, value) {
  if (!map || typeof map !== 'object') {
    return undefined;
  }
  const groupKey = Object.keys(map).find(
    (key) => key.toLowerCase() === String(name).toLowerCase(),
  );
  const group = groupKey ? map[groupKey] : null;
  if (!group || typeof group !== 'object') {
    return undefined;
  }
  const valueKey = Object.keys(group).find(
    (key) => key.toLowerCase() === String(value).toLowerCase(),
  );
  const count = valueKey ? group[valueKey] : undefined;
  return typeof count === 'number' ? count : undefined;
}

function setFilterCount(input, count) {
  const label = input.closest('label');
  const span = label?.querySelector('.pf-filter-checkbox-count');
  if (!label || !span) {
    return;
  }
  const next = String(count);
  if (span.textContent !== next) {
    span.textContent = next;
  }
  const show = count > 0 || input.checked;
  if (label.hasAttribute('data-pf-hidden') === show) {
    label.toggleAttribute('data-pf-hidden', !show);
  }
  const group = input.closest('details');
  if (group instanceof HTMLElement && show && group.hasAttribute('data-pf-hidden')) {
    group.removeAttribute('data-pf-hidden');
  }
}

function scheduleFilterOnlyCounts() {
  const instance = getPagefindInstance();
  const term = (instance?.searchTerm || '').trim();
  if (!instance || !term) {
    if (filterOnlyCounts) {
      filterOnlyCounts = null;
      filterOnlyCountsKey = '';
      scheduleRefreshFacetPanes();
    }
    return;
  }
  const key = filterCountsKey(instance.searchFilters);
  if (key === filterOnlyCountsKey && filterOnlyCounts) {
    scheduleRefreshFacetPanes();
    return;
  }
  const pagefind = instance.__pagefind__;
  if (!pagefind || typeof pagefind.search !== 'function') {
    return;
  }
  const request = ++filterOnlyRequest;
  const filters = normalizeSearchFilters(instance.searchFilters);
  pagefind.search(null, { filters }).then((result) => {
    if (request !== filterOnlyRequest) {
      return;
    }
    filterOnlyCounts = result?.filters || {};
    filterOnlyCountsKey = key;
    scheduleRefreshFacetPanes();
  }).catch(() => {});
}

function correctSelectedFilterCounts(pane) {
  if (!(pane instanceof HTMLElement)) {
    return;
  }
  const instance = getPagefindInstance();
  const term = (instance?.searchTerm || '').trim();
  const countsMatchFilters = filterCountsKey(instance?.searchFilters) === filterOnlyCountsKey;
  if (term && filterOnlyCounts && countsMatchFilters) {
    pane.querySelectorAll('input.pf-checkbox-input').forEach((input) => {
      if (!(input instanceof HTMLInputElement)) {
        return;
      }
      const count = countInFilterMap(filterOnlyCounts, input.name, input.value) ?? 0;
      setFilterCount(input, count);
    });
    return;
  }
  if (!pane.availableFilters) {
    return;
  }
  pane.querySelectorAll('input.pf-checkbox-input:checked').forEach((input) => {
    if (!(input instanceof HTMLInputElement)) {
      return;
    }
    const totalCount = countInFilterMap(pane.totalFilters, input.name, input.value) ?? 0;
    const availableCount = countInFilterMap(pane.availableFilters, input.name, input.value);
    if (totalCount > 0 || typeof availableCount !== 'number') {
      return;
    }
    setFilterCount(input, availableCount);
  });
}

const facetPaneObserved = new WeakSet();
let facetPaneRefreshRaf = null;
let facetPaneListenerRegistered = false;

function searchScopes() {
  return [document.querySelector('pagefind-modal'), document.querySelector('#ros-search-page')].filter(Boolean);
}

function refreshFacetPanes() {
  const instance = getPagefindInstance();
  const totalFilters = instance?.totalFilters ?? null;
  searchScopes().forEach((scope) => {
    scope.querySelectorAll('pagefind-filter-pane').forEach((pane) => {
      customizeFilterPane(pane, totalFilters);
      correctSelectedFilterCounts(pane);
    });
    ensureAlwaysShowPlaceholders(scope, totalFilters);
  });
}

function scheduleRefreshFacetPanes() {
  if (facetPaneRefreshRaf !== null) {
    return;
  }
  facetPaneRefreshRaf = requestAnimationFrame(() => {
    facetPaneRefreshRaf = null;
    refreshFacetPanes();
  });
}

function bindFacetPaneObservers(scopeEl) {
  if (!(scopeEl instanceof Element)) {
    return;
  }
  scopeEl.querySelectorAll('pagefind-filter-pane').forEach((pane) => {
    if (!(pane instanceof HTMLElement) || facetPaneObserved.has(pane)) {
      return;
    }
    const root = filterPaneRoot(pane);
    if (!root) {
      return;
    }
    facetPaneObserved.add(pane);
    const observer = new MutationObserver(() => {
      scheduleRefreshFacetPanes();
    });
    observer.observe(root, { childList: true, subtree: true });
    scheduleRefreshFacetPanes();
  });
}

function setupFacetPaneCustomization() {
  const instance = getPagefindInstance();
  if (!instance) {
    return;
  }
  if (!facetPaneListenerRegistered) {
    facetPaneListenerRegistered = true;
    instance.on('filters', () => {
      scheduleFilterOnlyCounts();
      scheduleRefreshFacetPanes();
    });
  }
  searchScopes().forEach((scope) => {
    bindFacetPaneObservers(scope);
  });
  scheduleRefreshFacetPanes();
}

const shadowMetaObservedHosts = new WeakSet();
let refreshMetaRaf = null;

function collectAllPfResultLis() {
  const seen = new Set();
  document.querySelectorAll('li.pf-result').forEach((el) => {
    if (el instanceof HTMLElement) {
      seen.add(el);
    }
  });
  return seen;
}

function refreshResultMetadata() {
  collectAllPfResultLis().forEach((item) => {
    void decorateResultItem(item);
  });
}

function scheduleRefreshResultMetadata() {
  if (refreshMetaRaf !== null) {
    return;
  }
  refreshMetaRaf = requestAnimationFrame(() => {
    refreshMetaRaf = null;
    refreshResultMetadata();
  });
}

function bindResultObserversForTarget(scopeEl) {
  if (!(scopeEl instanceof Element)) {
    return;
  }
  scopeEl.querySelectorAll('pagefind-results').forEach((host) => {
    if (!(host instanceof HTMLElement) || shadowMetaObservedHosts.has(host)) {
      return;
    }
    shadowMetaObservedHosts.add(host);
    const observer = new MutationObserver(() => {
      scheduleRefreshResultMetadata();
    });
    observer.observe(host, { childList: true, subtree: true });
    scheduleRefreshResultMetadata();
  });
}

function setupResultMetadataRendering() {
  searchScopes().forEach((target) => {
    if (target.dataset.metaEnhanced === '1') {
      return;
    }
    target.dataset.metaEnhanced = '1';
    bindResultObserversForTarget(target);
    bindFacetPaneObservers(target);
    const observer = new MutationObserver(() => {
      bindResultObserversForTarget(target);
      bindFacetPaneObservers(target);
      scheduleRefreshResultMetadata();
      scheduleRefreshFacetPanes();
    });
    observer.observe(target, { childList: true, subtree: true });
    scheduleRefreshResultMetadata();
  });
}

function distributionFacetAvailable() {
  if (!currentDistro || !isConfiguredFacetKey('distribution')) {
    return false;
  }
  const instance = getPagefindInstance();
  if (!instance) {
    return false;
  }
  const wanted = String(currentDistro).toLowerCase();
  for (const map of [instance.availableFilters, instance.totalFilters]) {
    if (!map || typeof map !== 'object') {
      continue;
    }
    const key = Object.keys(map).find((name) => name.toLowerCase() === 'distribution');
    const values = key ? map[key] : null;
    if (!values || typeof values !== 'object') {
      continue;
    }
    if (Object.keys(values).some((value) => String(value).toLowerCase() === wanted)) {
      return true;
    }
  }
  return false;
}

function mergeDefaultDistributionFilter(filters) {
  const merged = { ...filters };
  if (distributionFacetAvailable() && !merged.distribution?.length) {
    merged.distribution = [currentDistro];
  }
  return merged;
}

let defaultDistributionPinned = false;

function applyDefaultDistributionFilter() {
  if (defaultDistributionPinned) {
    return;
  }
  const instance = getPagefindInstance();
  if (!instance || !distributionFacetAvailable()) {
    return;
  }
  if (document.querySelector('#ros-search-page') && urlHasFacetParams()) {
    defaultDistributionPinned = true;
    return;
  }
  const existing = normalizeSearchFilters(instance.searchFilters);
  if (existing.distribution?.length) {
    defaultDistributionPinned = true;
    return;
  }
  defaultDistributionPinned = true;
  instance.triggerFilter('distribution', [currentDistro]);
}

function bindDefaultDistributionToModal() {
  const trigger = document.querySelector('pagefind-modal-trigger');
  if (trigger instanceof HTMLElement && !trigger.dataset.rosDistroDefaultBound) {
    trigger.dataset.rosDistroDefaultBound = '1';
    trigger.addEventListener('click', () => {
      defaultDistributionPinned = false;
      applyDefaultDistributionFilter();
    });
  }
  const dialog = document.querySelector('pagefind-modal dialog');
  if (dialog instanceof HTMLDialogElement && !dialog.dataset.rosDistroDefaultBound) {
    dialog.dataset.rosDistroDefaultBound = '1';
    dialog.addEventListener('close', () => {
      defaultDistributionPinned = false;
    });
  }
}

let userEditedPageSearchTerm = false;
let restoringUrlTerm = false;

function urlSearchTerm() {
  return (new URLSearchParams(window.location.search).get('q') || '').trim();
}

function reflectSearchTermOnPage(term) {
  const host = document.querySelector('#ros-search-page pagefind-input');
  const input = host?.querySelector('input');
  if (!(input instanceof HTMLInputElement)) {
    return false;
  }
  if (!input.dataset.rosTermWatch) {
    input.dataset.rosTermWatch = '1';
    input.addEventListener('input', () => {
      userEditedPageSearchTerm = true;
    });
  }
  if (input.value !== term) {
    input.value = term;
  }
  if (typeof host.updateState === 'function') {
    host.updateState(term);
  }
  return true;
}

function restoreUrlSearchTerm() {
  if (userEditedPageSearchTerm || restoringUrlTerm) {
    return;
  }
  if (!document.querySelector('#ros-search-page')) {
    return;
  }
  const term = urlSearchTerm();
  const instance = getPagefindInstance();
  if (!term || !instance) {
    return;
  }
  if ((instance.searchTerm || '').trim() !== term) {
    restoringUrlTerm = true;
    try {
      instance.triggerSearchWithFilters(
        term,
        mergeDefaultDistributionFilter(parseFiltersFromUrl()),
      );
    } finally {
      restoringUrlTerm = false;
    }
  }
  reflectSearchTermOnPage(term);
}

function applySearchQueryFromUrl() {
  const root = document.querySelector('#ros-search-page');
  if (!root) {
    return;
  }
  const params = new URLSearchParams(window.location.search);
  const term = (params.get('q') || '').trim();
  const urlFilters = parseFiltersFromUrl();
  const instance = getPagefindInstance();
  if (!instance) {
    return;
  }
  const filters = mergeDefaultDistributionFilter(urlFilters);
  const hasFilters = Object.keys(filters).length > 0;
  if (!term && !hasFilters) {
    return;
  }
  if (term) {
    instance.triggerSearchWithFilters(term, filters);
    reflectSearchTermOnPage(term);
    queueMicrotask(() => {
      restoreUrlSearchTerm();
    });
    return;
  }
  // Filter-only URL: apply the filters for the sidebar, but keep the result
  // list hidden until a search term is present.
  if (hasFilters) {
    instance.triggerSearchWithFilters('', filters);
  }
}

function syncResultListVisibility(term) {
  const hasTerm = Boolean(String(term ?? getPagefindInstance()?.searchTerm ?? '').trim());
  document.querySelectorAll('pagefind-modal, #ros-search-page').forEach((el) => {
    el.classList.toggle('ros-search-awaiting-term', !hasTerm);
  });
}

function liftSearchModalOutOfSidebar() {
  // The trigger lives in .wy-nav-side (fixed, z-index 200, overflow hidden).
  // A modal dialog kept inside that sidebar paints over the page, but the
  // filter column still overlaps the sidebar layer, so clicks miss the checkboxes.
  const host = document.querySelector('.ros2-pagefind-search pagefind-modal');
  if (!(host instanceof HTMLElement) || host.closest('.ros2-pagefind-search--portal')) {
    return;
  }
  const portal = document.createElement('div');
  portal.className = 'ros2-pagefind-search ros2-pagefind-search--portal ros-search-ui';
  document.body.appendChild(portal);
  portal.appendChild(host);
}

async function bootPagefind() {
  if (!config || !componentJs) {
    return;
  }
  const configEl = document.createElement('pagefind-config');
  configEl.setAttribute('bundle-path', new URL(bundlePrefix, document.baseURI).href);
  // Allow filter-only searches (empty query + filters) instead of clearing results.
  configEl.setAttribute('faceted', '');
  document.head.appendChild(configEl);

  bindModalEnterToSearchPage();
  liftSearchModalOutOfSidebar();
  await import(new URL(componentJs, document.baseURI).href);
  setupFacetPaneCustomization();
  setupResultMetadataRendering();
  bindDefaultDistributionToModal();
  applySearchQueryFromUrl();
  applyDefaultDistributionFilter();
  syncResultListVisibility();
  const instance = getPagefindInstance();
  if (instance?.on) {
    instance.on('search', (term) => {
      syncResultListVisibility(term);
      scheduleFilterOnlyCounts();
      scheduleRefreshResultMetadata();
      applyDefaultDistributionFilter();
      if (userEditedPageSearchTerm || restoringUrlTerm || !urlSearchTerm()) {
        return;
      }
      if ((term || '') !== urlSearchTerm()) {
        queueMicrotask(() => {
          restoreUrlSearchTerm();
        });
        return;
      }
      reflectSearchTermOnPage(urlSearchTerm());
    });
    instance.on('filters', () => {
      applyDefaultDistributionFilter();
    });
  }
}

if (config) {
  bootPagefind().catch((err) => {
    console.warn('Pagefind UI unavailable until Pagefind CLI is run on the build output:', err);
  });
}
