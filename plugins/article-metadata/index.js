const fs = require('node:fs/promises');
const path = require('node:path');
const yaml = require('js-yaml');

module.exports = function articleMetadata(context) {
  return {
    name: 'article-metadata',
    getPathsToWatch() {
      return [path.join(context.siteDir, 'blog/**/*.{md,mdx}')];
    },
    async loadContent() {
      const posts = [];

      async function walk(dir) {
        for (const entry of await fs.readdir(dir, {withFileTypes: true})) {
          const file = path.join(dir, entry.name);
          if (entry.isDirectory()) {
            await walk(file);
            continue;
          }
          if (!/\.mdx?$/.test(entry.name)) continue;

          const raw = await fs.readFile(file, 'utf8');
          const match = raw.match(/^---\r?\n([\s\S]*?)\r?\n---/);
          const meta = match ? yaml.load(match[1]) : {};
          if (meta.draft || meta.unlisted || !meta.title) continue;

          const relative = path.relative(path.join(context.siteDir, 'blog'), file).replace(/\\/g, '/').replace(/\.mdx?$/, '');
          const date = relative.match(/^\d{4}-\d{2}-\d{2}/)?.[0] || '';
          if (date && date > new Date().toISOString().slice(0, 10)) continue;
          const slug = meta.slug || relative.replace(/\/index$/, '').replace(/^\d{4}-\d{2}-\d{2}-/, '');

          posts.push({
            title: meta.title,
            url: `/blog/${slug}`,
            description: meta.description || '',
            date,
            youtube: meta.youtube,
          });
        }
      }

      await walk(path.join(context.siteDir, 'blog'));
      return posts.sort((a, b) => b.date.localeCompare(a.date));
    },
    async contentLoaded({content, actions}) {
      actions.setGlobalData({posts: content});
    },
  };
};
