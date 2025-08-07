const { chromium } = require('playwright');

async function scrapeDetailedExample() {
  const browser = await chromium.launch();
  const page = await browser.newPage();
  
  try {
    console.log('Scraping detailed Rete.js v2 example...');
    
    // Visit the basic guide and wait for content to load
    await page.goto('https://retejs.org/docs/guides/basic/', { timeout: 30000 });
    await page.waitForSelector('pre code', { timeout: 10000 });
    
    // Get all code blocks
    const codeBlocks = await page.$$eval('pre code', codes => 
      codes.map(code => code.textContent.trim())
    );
    
    console.log('=== ALL CODE EXAMPLES FROM BASIC GUIDE ===');
    codeBlocks.forEach((code, index) => {
      if (code.length > 10) { // Filter out very short snippets
        console.log(`\n--- Code Block ${index + 1} ---`);
        console.log(code);
      }
    });
    
    // Also try to get the full working example
    await page.goto('https://codesandbox.io/s/yrzxe5', { timeout: 30000 });
    await page.waitForTimeout(3000);
    
    // Look for the main app file
    const mainFileCode = await page.$eval('iframe[name="codesandbox"]', iframe => {
      return iframe.contentDocument?.body?.textContent || 'Could not access iframe content';
    }).catch(() => 'Could not access CodeSandbox content');
    
    console.log('\n=== CODESANDBOX CONTENT ===');
    console.log(mainFileCode.substring(0, 1000));
    
  } catch (error) {
    console.error('Error:', error.message);
  } finally {
    await browser.close();
  }
}

scrapeDetailedExample();