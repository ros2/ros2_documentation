/**
 * Populate ``.js-related-packages`` widgets from the rosdistro cache YAML.
 *
 * Depends on global ``pako`` (gzip) and ``yaml`` / ``jsyaml`` (js-yaml), loaded
 * earlier via html_js_files in conf.py.
 */
(function () {
  'use strict';

  /** @type {Record<string, Promise<Record<string, string>>>} */
  var cacheByDistro = {};

  /**
   * Resolve the js-yaml API regardless of how the bundle exposes it.
   *
   * @returns {{ load: function(string): unknown }}
   */
  function yamlApi() {
    var g = typeof window !== 'undefined' ? window : globalThis;
    /* js-yaml UMD sets ``globalThis.jsyaml`` (see dist/js-yaml.min.js). */
    if (g.jsyaml && typeof g.jsyaml.load === 'function') {
      return g.jsyaml;
    }
    if (g.yaml && typeof g.yaml.load === 'function') {
      return g.yaml;
    }
    throw new Error('js-yaml is not loaded');
  }

  /**
   * Directory containing ``related_packages.js`` (ends with slash or empty).
   *
   * @returns {string}
   */
  function scriptBaseUrl() {
    var nodes = document.getElementsByTagName('script');
    var i;
    var src;
    for (i = nodes.length - 1; i >= 0; i--) {
      src = nodes[i].src;
      if (src && src.indexOf('related_packages.js') !== -1) {
        return src.replace(/related_packages\.js([?#].*)?$/i, '');
      }
    }
    return '';
  }

  /**
   * @param {string} distro
   * @returns {string|null}
   */
  function bundledCacheUrl(distro) {
    var base = scriptBaseUrl();
    if (!base) {
      return null;
    }
    return base + 'rosdistro_cache/' + distro + '-cache.yaml.gz';
  }

  /**
   * Prefer Sphinx-emitted ``data-bundled-cache-href`` (relative to page); then derive from script URL.
   *
   * @param {HTMLElement|null} widget
   * @param {string} distro
   * @returns {string|null}
   */
  function resolveBundledAbsoluteUrl(widget, distro) {
    var rel = widget && widget.getAttribute('data-bundled-cache-href');
    if (rel && typeof URL !== 'undefined') {
      try {
        return new URL(rel, window.location.href).href;
      } catch (e1) {
        /* ignore */
      }
    }
    return bundledCacheUrl(distro);
  }

  /**
   * Proxy URL configured by Sphinx via data attribute.
   *
   * @param {HTMLElement|null} widget
   * @param {string} distro
   * @returns {string|null}
   */
  function resolveProxyUrl(widget, distro) {
    var templateUrl = widget && widget.getAttribute('data-proxy-cache-href');
    if (!templateUrl) {
      return null;
    }
    return templateUrl.replace('{distro}', encodeURIComponent(distro));
  }

  /**
   * @param {string} distro
   * @param {HTMLElement|null} sampleWidget widget from this page (for data-bundled-cache-href)
   * @returns {Promise<Record<string, string>>}
   */
  function loadXmls(distro, sampleWidget) {
    var cacheKey =
      distro +
      '|' +
      (sampleWidget ? sampleWidget.getAttribute('data-proxy-cache-href') || '' : '') +
      '|' +
      (sampleWidget ? sampleWidget.getAttribute('data-bundled-cache-href') || '' : '');
    if (cacheByDistro[cacheKey]) {
      return cacheByDistro[cacheKey];
    }
    cacheByDistro[cacheKey] = fetchAndParse(
      distro,
      resolveProxyUrl(sampleWidget, distro),
      resolveBundledAbsoluteUrl(sampleWidget, distro)
    );
    return cacheByDistro[cacheKey];
  }

  /**
   * @param {string} distro
   * @param {string|null} proxyUrl same-origin backend proxy endpoint (freshest)
   * @param {string|null} bundledAbsolute resolved same-origin URL to gzip, if any
   * @returns {Promise<Record<string, string>>}
   */
  function fetchAndParse(distro, proxyUrl, bundledAbsolute) {
    var remote =
      'https://repo.ros2.org/rosdistro_cache/' + encodeURIComponent(distro) + '-cache.yaml.gz';
    var urls = [];
    if (proxyUrl) {
      urls.push(proxyUrl);
    }
    if (bundledAbsolute) {
      urls.push(bundledAbsolute);
    }
    /* Final fallback may still fail in browsers due to upstream CORS. */
    urls.push(remote);

    return tryUrls(urls);
  }

  /**
   * @param {string[]} urls
   * @returns {Promise<Record<string, string>>}
   */
  function tryUrls(urls) {
    var i = 0;

    function next(lastErr) {
      if (i >= urls.length) {
        return Promise.reject(lastErr || new Error('failed to load rosdistro cache'));
      }
      var url = urls[i];
      i += 1;
      var controller = typeof AbortController !== 'undefined' ? new AbortController() : null;
      var timer = null;
      if (controller && i === 1) {
        /* Keep proxy attempt snappy so fallback isn't delayed. */
        timer = setTimeout(function () {
          controller.abort();
        }, 6000);
      }
      return fetch(url, { cache: 'no-cache', signal: controller ? controller.signal : undefined })
        .then(function (res) {
          if (timer) {
            clearTimeout(timer);
          }
          if (!res.ok) {
            throw new Error('HTTP ' + res.status + ' for ' + url);
          }
          return res.arrayBuffer();
        })
        .then(function (buf) {
          var g = typeof window !== 'undefined' ? window : globalThis;
          var inflated = g.pako.inflate(new Uint8Array(buf), { to: 'string' });
          var data = yamlApi().load(inflated);
          var xmls = data && data.release_package_xmls;
          if (!xmls || typeof xmls !== 'object') {
            throw new Error('release_package_xmls missing in rosdistro cache');
          }
          if (typeof console !== 'undefined' && console.info) {
            console.info('related_packages: loaded rosdistro cache from', url);
          }
          return /** @type {Record<string, string>} */ (xmls);
        })
        .catch(function (err) {
          if (timer) {
            clearTimeout(timer);
          }
          if (typeof console !== 'undefined' && console.warn) {
            console.warn('related_packages: failed', url, err);
          }
          /* Try next URL (e.g. bundled 404 then HTTPS remote — remote may hit CORS). */
          return next(err);
        });
    }

    return next(null);
  }

  /**
   * @param {string} xmlStr
   * @returns {string[]}
   */
  function extractBuildTypes(xmlStr) {
    var out = [];
    var re = /<build_type[^>]*>([^<]+)<\/build_type>/gi;
    var m;
    while ((m = re.exec(xmlStr)) !== null) {
      out.push(m[1].trim());
    }
    return out;
  }

  /**
   * @param {string} xmlStr
   * @param {string} want
   * @returns {boolean}
   */
  function matchesBuildType(xmlStr, want) {
    var types = extractBuildTypes(xmlStr);
    var k;
    for (k = 0; k < types.length; k += 1) {
      if (types[k] === want) {
        return true;
      }
    }
    return false;
  }

  /**
   * @param {string} xmlStr
   * @returns {string}
   */
  function extractDescription(xmlStr) {
    if (typeof DOMParser !== 'undefined') {
      try {
        var doc = new DOMParser().parseFromString(xmlStr, 'application/xml');
        var parseErr = doc.getElementsByTagName('parsererror');
        if (!parseErr.length) {
          var nodes = doc.getElementsByTagName('description');
          if (nodes.length && nodes[0].textContent) {
            return nodes[0].textContent.replace(/\s+/g, ' ').trim();
          }
        }
      } catch (err) {
        /* Fall through to regex extraction. */
      }
    }

    var match = /<description\b[^>]*>([\s\S]*?)<\/description>/i.exec(xmlStr);
    if (!match) {
      return '';
    }
    return match[1]
      .replace(/<!\[CDATA\[([\s\S]*?)\]\]>/g, '$1')
      .replace(/<[^>]*>/g, ' ')
      .replace(/\s+/g, ' ')
      .trim();
  }

  /**
   * @param {string} distro
   * @param {string} pkg
   * @returns {string}
   */
  function docsPackageUrl(distro, pkg) {
    return (
      'https://docs.ros.org/en/' +
      encodeURIComponent(distro) +
      '/p/' +
      encodeURIComponent(pkg) +
      '/'
    );
  }

  /**
   * @param {HTMLElement} el
   * @param {Error} err
   */
  function showError(el, err) {
    el.classList.remove('related-packages--loading');
    el.classList.add('related-packages--error');
    el.innerHTML =
      '<p class="related-packages__status">Could not load package metadata. ' +
      'Rebuild the HTML documentation while online so the rosdistro cache is ' +
      'downloaded into <code>_static/rosdistro_cache/</code>, ' +
      'or check your network connection.</p>';
    if (typeof console !== 'undefined' && console.warn) {
      console.warn('related_packages:', err);
    }
  }

  /**
   * @param {HTMLElement} el
   * @param {Record<string, string>} xmls
   */
  function fillWidget(el, xmls) {
    var want = el.getAttribute('data-build-type') || '';
    var max = parseInt(el.getAttribute('data-max') || '10', 10);
    var distro = el.getAttribute('data-distro') || 'rolling';

    var names = Object.keys(xmls).filter(function (name) {
      var xmlStr = xmls[name];
      if (typeof xmlStr !== 'string') {
        return false;
      }
      return matchesBuildType(xmlStr, want);
    });
    names.sort(function (a, b) {
      return a.localeCompare(b);
    });
    var picked = names.slice(0, max);

    var ul = document.createElement('ul');
    ul.className = 'related-packages__list';
    var j;
    for (j = 0; j < picked.length; j += 1) {
      var pkg = picked[j];
      var li = document.createElement('li');
      var a = document.createElement('a');
      var description = extractDescription(xmls[pkg] || '');
      a.href = docsPackageUrl(distro, pkg);
      a.textContent = pkg;
      a.rel = 'noopener noreferrer';
      li.appendChild(a);
      li.appendChild(document.createTextNode(': ' + description));
      ul.appendChild(li);
    }

    el.innerHTML = '';
    el.classList.remove('related-packages--loading');

    if (picked.length === 0) {
      var p = document.createElement('p');
      p.className = 'related-packages__empty';
      p.textContent = 'No packages matched this filter.';
      el.appendChild(p);
    } else {
      var intro = document.createElement('p');
      intro.className = 'related-packages__intro';
      intro.textContent = 'Packages/reference: ';
      el.appendChild(intro);
      el.appendChild(ul);
    }
  }

  function fillAll() {
    var widgets = document.querySelectorAll('.js-related-packages');
    if (!widgets.length) {
      return;
    }

    /** @type {Record<string, HTMLElement[]>} */
    var byDistro = {};
    var idx;
    for (idx = 0; idx < widgets.length; idx += 1) {
      var el = widgets[idx];
      var d = el.getAttribute('data-distro') || 'rolling';
      if (!byDistro[d]) {
        byDistro[d] = [];
      }
      byDistro[d].push(el);
    }

    var distroKeys = Object.keys(byDistro);
    var di;
    for (di = 0; di < distroKeys.length; di += 1) {
      (function (distro) {
        var group = byDistro[distro];
        loadXmls(distro, group[0]).then(
          function (xmls) {
            var gi;
            for (gi = 0; gi < group.length; gi += 1) {
              fillWidget(group[gi], xmls);
            }
          },
          function (err) {
            var ei;
            for (ei = 0; ei < group.length; ei += 1) {
              showError(group[ei], err);
            }
          }
        );
      })(distroKeys[di]);
    }
  }

  if (typeof document !== 'undefined') {
    if (document.readyState === 'loading') {
      document.addEventListener('DOMContentLoaded', fillAll);
    } else {
      fillAll();
    }
  }
})();
