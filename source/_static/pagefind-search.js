// Pagefind search UI for the docs sidebar modal and search.html.
// Facet labels, the current distro, and bundle paths come from #ros-pagefind-config.
// DOM hooks below target the Pagefind Component UI pinned by PAGEFIND_VERSION in the Makefile.

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
const facetKeys = resultMetaFields.map((field) => field.key);
const labelByKey = new Map(resultMetaFields.map((field) => [field.key, field.label || field.key]));
const alwaysShowFilterKeys = new Set(Array.isArray(config?.alwaysShowFilterKeys) ? config.alwaysShowFilterKeys : []);
const currentDistro = String(config?.currentDistro || '');
const searchResultsUrl = String(config?.searchResultsUrl || 'search.html');
const bundlePrefix = String(config?.bundlePrefix || './pagefind/');
const showBreadcrumb = Boolean(config?.showBreadcrumb);
const showMetadata = Boolean(config?.showMetadata);
const excerptLength = config?.excerptLength ?? 30;

function getPagefindInstance() {
  return window.PagefindComponents?.getInstanceManager?.()?.getInstance?.('default') ?? null;
}

function cleanValues(values) {
  const arr = Array.isArray(values) ? values : [values];
  return arr.map((value) => String(value ?? '').trim()).filter(Boolean);
}

function normalizeSearchFilters(raw) {
  const out = {};
  if (!raw || typeof raw !== 'object') {
    return out;
  }
  for (const key of facetKeys) {
    const cleaned = cleanValues(raw[key]);
    if (cleaned.length) {
      out[key] = cleaned;
    }
  }
  return out;
}

function parseFiltersFromUrl() {
  const params = new URLSearchParams(window.location.search);
  const filters = {};
  for (const key of facetKeys) {
    const values = cleanValues(params.getAll(key));
    if (values.length) {
      filters[key] = values;
    }
  }
  return filters;
}

function urlHasFacetParams() {
  return Object.keys(parseFiltersFromUrl()).length > 0;
}

function buildSearchResultsHref(term, searchFilters) {
  const params = new URLSearchParams();
  const query = (term || '').trim();
  if (query) {
    params.set('q', query);
  }
  const filters = normalizeSearchFilters(searchFilters);
  for (const key of facetKeys) {
    (filters[key] || []).forEach((value) => params.append(key, value));
  }
  const encoded = params.toString();
  return encoded ? `${searchResultsUrl}?${encoded}` : searchResultsUrl;
}

function getModalSearchTerm(event) {
  const fromInstance = (getPagefindInstance()?.searchTerm || '').trim();
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

// Result cards: breadcrumb and metadata come straight from each result's data.

function metadataEntries(data) {
  if (!showMetadata) {
    return [];
  }
  return facetKeys
    .map((key) => ({ key, value: cleanValues(data?.filters?.[key] ?? data?.meta?.[key]).join(', ') }))
    .filter((entry) => entry.value);
}

function buildMetadataBlock(entries) {
  const block = document.createElement('p');
  block.className = 'pf-result-meta-block ros-page-meta-summary';
  entries.forEach(({ key, value }, index) => {
    if (index > 0) {
      block.appendChild(document.createTextNode(' | '));
    }
    const nameEl = document.createElement('strong');
    const label = labelByKey.get(key) || key;
    nameEl.textContent = label.endsWith(':') ? label : `${label}:`;
    block.appendChild(nameEl);
    block.appendChild(document.createTextNode(` ${value}`));
  });
  return block;
}

function buildBreadcrumbEl(text) {
  const el = document.createElement('p');
  el.className = 'pf-result-breadcrumb';
  const label = document.createElement('span');
  label.className = 'pf-result-breadcrumb-label';
  label.textContent = 'In:';
  el.appendChild(label);
  el.appendChild(document.createTextNode(` ${text}`));
  return el;
}

function insertResultChrome(content, data) {
  const nodes = [];
  const breadcrumb = showBreadcrumb ? String(data?.meta?.breadcrumb || '').trim() : '';
  if (breadcrumb) {
    nodes.push(buildBreadcrumbEl(breadcrumb));
  }
  const entries = metadataEntries(data);
  if (entries.length) {
    nodes.push(buildMetadataBlock(entries));
  }
  const title = content.querySelector(':scope > .pf-result-title');
  if (title) {
    title.after(...nodes);
  } else {
    content.prepend(...nodes);
  }
}

function toElements(output) {
  if (output instanceof Element) {
    return [output];
  }
  if (Array.isArray(output)) {
    return output.filter((node) => node instanceof Element);
  }
  const tpl = document.createElement('template');
  tpl.innerHTML = String(output ?? '');
  return [...tpl.content.children];
}

// Wrap <pagefind-results>' default card renderer to add the breadcrumb and metadata.
function decorateResultRenderer(host) {
  if (host.dataset.rosRendererWrapped || typeof host.getResultRenderer !== 'function') {
    return;
  }
  host.dataset.rosRendererWrapped = '1';
  const original = host.getResultRenderer.bind(host);
  host.getResultRenderer = () => {
    const render = original();
    return (data, options) => {
      const elements = toElements(render(data, options));
      elements.forEach((el) => {
        const content = el.querySelector('.pf-result-content');
        if (content) {
          insertResultChrome(content, data);
        }
      });
      return elements;
    };
  };
}

// Facet sidebar: hide unconfigured groups, apply labels and order, add placeholders.

function customizeFilterPane(pane) {
  const root = pane.querySelector('.pf-filter-pane') || pane;
  const matched = new Map();
  root.querySelectorAll('.pf-filter-group').forEach((group) => {
    const key = group.dataset.filterName;
    const show = labelByKey.has(key);
    const nextDisplay = show ? '' : 'none';
    if (group.style.display !== nextDisplay) {
      group.style.display = nextDisplay;
    }
    if (!show) {
      return;
    }
    const title = labelByKey.get(key);
    group.querySelectorAll('.pf-filter-group-name').forEach((el) => {
      if (el.textContent !== title) {
        el.textContent = title;
      }
    });
    matched.set(key, group);
  });
  const ordered = facetKeys.map((key) => matched.get(key)).filter(Boolean);
  const container = ordered[0]?.parentElement;
  if (!container) {
    return;
  }
  if (!ordered.every((el, index) => container.children[index] === el)) {
    ordered.forEach((el) => container.appendChild(el));
  }
}

function ensureAlwaysShowPlaceholders(scopeEl, totalFilters) {
  const hasIndexedFilters = Boolean(totalFilters) && Object.keys(totalFilters).length > 0;
  const missingKeys = facetKeys.filter(
    (key) => alwaysShowFilterKeys.has(key) && !Object.hasOwn(totalFilters || {}, key),
  );

  scopeEl.querySelectorAll('.ros-search-facets').forEach((aside) => {
    aside.classList.toggle('ros-search-facets--placeholders', !hasIndexedFilters);
    aside.classList.toggle('ros-search-facets--native', hasIndexedFilters);
    aside.querySelectorAll('pagefind-filter-pane').forEach((pane) => {
      pane.hidden = !hasIndexedFilters;
    });

    let host = aside.querySelector('.ros-facet-placeholders');
    if (!host) {
      host = document.createElement('div');
      host.className = 'ros-facet-placeholders';
      aside.appendChild(host);
    }
    host.hidden = missingKeys.length === 0;

    [...host.querySelectorAll('[data-ros-facet-placeholder]')].forEach((el) => {
      if (!missingKeys.includes(el.dataset.rosFacetPlaceholder)) {
        el.remove();
      }
    });

    const placeholders = missingKeys.map((key) => {
      let details = host.querySelector(`[data-ros-facet-placeholder="${CSS.escape(key)}"]`);
      if (!details) {
        details = document.createElement('details');
        details.className = 'ros-facet-placeholder';
        details.dataset.rosFacetPlaceholder = key;
        details.open = true;
        const summary = document.createElement('summary');
        summary.textContent = labelByKey.get(key);
        const empty = document.createElement('p');
        empty.className = 'ros-facet-placeholder-empty';
        empty.textContent = 'No options available';
        details.append(summary, empty);
        host.appendChild(details);
      }
      return details;
    });
    if (!placeholders.every((el, index) => host.children[index] === el)) {
      placeholders.forEach((el) => host.appendChild(el));
    }
  });
}

let facetPaneRefreshRaf = null;

function searchScopes() {
  return [document.querySelector('pagefind-modal'), document.querySelector('#ros-search-page')].filter(Boolean);
}

function refreshSearchScopes() {
  const instance = getPagefindInstance();
  useOrWithinFilterGroups(instance);
  const totalFilters = instance?.totalFilters ?? null;
  searchScopes().forEach((scope) => {
    scope.querySelectorAll('pagefind-filter-pane').forEach(customizeFilterPane);
    scope.querySelectorAll('pagefind-results').forEach(decorateResultRenderer);
    ensureAlwaysShowPlaceholders(scope, totalFilters);
  });
}

function scheduleRefreshSearchScopes() {
  if (facetPaneRefreshRaf !== null) {
    return;
  }
  facetPaneRefreshRaf = requestAnimationFrame(() => {
    facetPaneRefreshRaf = null;
    refreshSearchScopes();
  });
}

function observeSearchScopes() {
  searchScopes().forEach((scope) => {
    new MutationObserver(scheduleRefreshSearchScopes).observe(scope, { childList: true, subtree: true });
  });
  refreshSearchScopes();
}

// Default the Distribution facet to the distro this page was built for.

function distributionFacetAvailable() {
  if (!currentDistro || !labelByKey.has('distribution')) {
    return false;
  }
  const instance = getPagefindInstance();
  return [instance?.availableFilters, instance?.totalFilters].some(
    (map) => map?.distribution && Object.hasOwn(map.distribution, currentDistro),
  );
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
  defaultDistributionPinned = true;
  if (document.querySelector('#ros-search-page') && urlHasFacetParams()) {
    return;
  }
  if (normalizeSearchFilters(instance.searchFilters).distribution?.length) {
    return;
  }
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

// search.html: run the ?q= search and keep the term in the input.

let userEditedPageSearchTerm = false;
let restoringUrlTerm = false;

function urlSearchTerm() {
  return (new URLSearchParams(window.location.search).get('q') || '').trim();
}

// Set a <pagefind-input>'s text and Pagefind's view of it; returns the <input>.
function setInputTerm(host, term) {
  const input = host?.querySelector('input');
  if (!(input instanceof HTMLInputElement)) {
    return null;
  }
  if (input.value !== term) {
    input.value = term;
  }
  if (typeof host.updateState === 'function') {
    host.updateState(term);
  }
  return input;
}

function reflectSearchTermOnPage(term) {
  const input = setInputTerm(document.querySelector('#ros-search-page pagefind-input'), term);
  if (input && !input.dataset.rosTermWatch) {
    input.dataset.rosTermWatch = '1';
    input.addEventListener('input', () => {
      userEditedPageSearchTerm = true;
    });
  }
}

function restoreUrlSearchTerm() {
  if (userEditedPageSearchTerm || restoringUrlTerm || !document.querySelector('#ros-search-page')) {
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
      instance.triggerSearchWithFilters(term, mergeDefaultDistributionFilter(parseFiltersFromUrl()));
    } finally {
      restoringUrlTerm = false;
    }
  }
  reflectSearchTermOnPage(term);
}

function applySearchQueryFromUrl() {
  const instance = getPagefindInstance();
  if (!document.querySelector('#ros-search-page') || !instance) {
    return;
  }
  const term = urlSearchTerm();
  const filters = mergeDefaultDistributionFilter(parseFiltersFromUrl());
  if (term) {
    instance.triggerSearchWithFilters(term, filters);
    reflectSearchTermOnPage(term);
    queueMicrotask(restoreUrlSearchTerm);
    return;
  }
  // Filter-only URL: apply the filters for the sidebar, but keep the result
  // list hidden until a search term is present.
  if (Object.keys(filters).length) {
    instance.triggerSearchWithFilters('', filters);
  }
}

function forgetSearchUrl() {
  if (!document.querySelector('#ros-search-page')) {
    return;
  }
  if (window.location.search) {
    history.replaceState(null, '', window.location.pathname + window.location.hash);
  }
}

function bindClearSearch() {
  // Pagefind's Clear button only empties the text. Capture the click and also
  // drop every selected filter. On search.html, drop the query string so the
  // term and filters are not restored from the URL.
  document.addEventListener('click', (event) => {
    const button = event.target instanceof Element
      ? event.target.closest('button.pf-input-clear')
      : null;
    if (!(button instanceof HTMLButtonElement)) {
      return;
    }
    event.preventDefault();
    event.stopPropagation();
    const input = setInputTerm(button.closest('pagefind-input'), '');
    userEditedPageSearchTerm = true;
    // Clearing is a deliberate choice: do not re-apply the default distro afterwards.
    defaultDistributionPinned = true;
    getPagefindInstance()?.triggerSearchWithFilters('', {});
    forgetSearchUrl();
    input?.focus();
  }, true);
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

// Pagefind treats an array of values as AND: the page must have every value.
// { any: [...] } is OR within that group. Separate groups stay AND with each
// other and with the search term.
function filtersWithAnyWithinGroups(filters) {
  return Object.fromEntries(
    Object.entries(normalizeSearchFilters(filters)).map(([key, values]) => [key, { any: values }]),
  );
}

function useOrWithinFilterGroups(instance) {
  if (!instance || instance.__rosOrWithinGroups || typeof instance.__search__ !== 'function') {
    return;
  }
  const search = instance.__search__.bind(instance);
  instance.__search__ = (term, filters) => search(term, filtersWithAnyWithinGroups(filters));
  instance.__rosOrWithinGroups = true;
}

async function bootPagefind() {
  const bundleUrl = new URL(bundlePrefix, document.baseURI);
  const configEl = document.createElement('pagefind-config');
  configEl.setAttribute('bundle-path', bundleUrl.href);
  configEl.setAttribute('excerpt-length', String(excerptLength));
  // Allow filter-only searches (empty query + filters) instead of clearing results.
  configEl.setAttribute('faceted', '');
  document.head.appendChild(configEl);

  bindModalEnterToSearchPage();
  bindClearSearch();
  liftSearchModalOutOfSidebar();
  await import(new URL('pagefind-component-ui.js', bundleUrl).href);
  observeSearchScopes();
  bindDefaultDistributionToModal();
  applySearchQueryFromUrl();
  applyDefaultDistributionFilter();
  syncResultListVisibility();
  const instance = getPagefindInstance();
  if (!instance?.on) {
    return;
  }
  instance.on('filters', () => {
    scheduleRefreshSearchScopes();
    applyDefaultDistributionFilter();
  });
  instance.on('search', (term) => {
    syncResultListVisibility(term);
    applyDefaultDistributionFilter();
    if (userEditedPageSearchTerm || restoringUrlTerm || !urlSearchTerm()) {
      return;
    }
    if ((term || '') !== urlSearchTerm()) {
      queueMicrotask(restoreUrlSearchTerm);
      return;
    }
    reflectSearchTermOnPage(urlSearchTerm());
  });
}

if (config) {
  bootPagefind().catch((err) => {
    console.warn('Pagefind UI unavailable until Pagefind CLI is run on the build output:', err);
  });
}
