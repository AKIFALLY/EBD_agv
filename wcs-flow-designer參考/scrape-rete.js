const { chromium } = require('playwright');

async function scrapeReteDocumentation() {
  const browser = await chromium.launch();
  const page = await browser.newPage();
  
  try {
    console.log('Scraping Rete.js v2 documentation...');
    
    // Visit the basic guide
    await page.goto('https://retejs.org/docs/guides/basic/', { timeout: 30000 });
    await page.waitForTimeout(2000);
    
    const basicContent = await page.textContent('body');
    console.log('=== BASIC GUIDE ===');
    console.log(basicContent.substring(0, 2000));
    
    // Visit the getting started page
    await page.goto('https://retejs.org/docs/getting-started/', { timeout: 30000 });
    await page.waitForTimeout(2000);
    
    const gettingStartedContent = await page.textContent('body');
    console.log('\n=== GETTING STARTED ===');
    console.log(gettingStartedContent.substring(0, 2000));
    
    // Visit the editor concepts page
    await page.goto('https://retejs.org/docs/concepts/editor/', { timeout: 30000 });
    await page.waitForTimeout(2000);
    
    const editorContent = await page.textContent('body');
    console.log('\n=== EDITOR CONCEPTS ===');
    console.log(editorContent.substring(0, 2000));
    
    // Look for code examples
    const codeBlocks = await page.$$('pre code, .highlight code');
    if (codeBlocks.length > 0) {
      console.log('\n=== CODE EXAMPLES ===');
      for (let i = 0; i < Math.min(3, codeBlocks.length); i++) {
        const code = await codeBlocks[i].textContent();
        console.log(`--- Example ${i + 1} ---`);
        console.log(code);
      }
    }
    
  } catch (error) {
    console.error('Error scraping:', error.message);
  } finally {
    await browser.close();
  }
}

scrapeReteDocumentation();