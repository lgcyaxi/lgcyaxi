'''
Author       : Lihao leolihao@arizona.edu
Date         : 2025-03-22 13:17:16
FilePath     : /EasyCalib-server/tools/clean_cache.py
Description  : 
Copyright (c) 2025 by Lihao (leolihao@arizona.edu), All Rights Reserved.
'''
import shutil
import os
import glob
import argparse
from pathlib import Path

# Enhanced Directory Cleaner
CLEAN_CONFIG = {
    # Need to be purged directories
    "purge_dirs": [
        '**/__pycache__',
        '.pytest_cache',
        'build',
        'dist',
        '.eggs',
    ],
    
    # Directories to be cleaned but kept
    "clean_inside_dirs": [
        '/tmp/staging/*'  # Support wildcards
    ],
    
    # File cleaning patterns
    "file_patterns": [
        '**/*.pyc',
        '**/*.pyo',
        '**/.DS_Store',
    ],
    
    # Deep cleaning extensions
    "deep_clean_additions": {
        'dirs': ['**/.ipynb_checkpoints'],
        'files': ['**/*.log',
                  './data/postgres/*',
                  './data/redis/*']
    }
}

class SmartCleaner:
    def __init__(self, dry_run=False):
        self.dry_run = dry_run
        self.stats = {
            'dirs_removed': 0,
            'files_removed': 0,
            'contents_cleaned': 0
        }
    
    def _resolve_patterns(self, patterns):
        """Parse and validate path patterns"""
        resolved = []
        for pattern in patterns:
            # Expand user directories and variables
            expanded = os.path.expanduser(os.path.expandvars(pattern))
            
            # Handle absolute path wildcards
            if expanded.startswith('/') and '*' in expanded:
                for path in glob.glob(expanded):
                    resolved.append(Path(path).resolve())
            else:
                resolved.extend([
                    Path(p).resolve() 
                    for p in glob.glob(expanded, recursive=True)
                ])
        return list(set(resolved))  # Remove duplicates

    def _clean_directory_contents(self, dir_path):
        """Clean directory contents but keep the directory itself"""
        try:
            entries = list(dir_path.glob('*'))
            if not entries:
                return
                
            print(f"{'Would clean' if self.dry_run else 'Cleaning'} contents of: {dir_path}")
            
            if self.dry_run:
                self.stats['contents_cleaned'] += len(entries)
                return
            
            for entry in entries:
                if entry.is_dir():
                    shutil.rmtree(entry, ignore_errors=True)
                    self.stats['dirs_removed'] += 1
                else:
                    entry.unlink(missing_ok=True)
                    self.stats['files_removed'] += 1
            self.stats['contents_cleaned'] += len(entries)
        except Exception as e:
            print(f"Error cleaning {dir_path}: {str(e)}")

    def _purge_directory(self, dir_path):
        """Purge directory"""
        print(f"{'Would remove' if self.dry_run else 'Removing'} directory: {dir_path}")
        
        if self.dry_run:
            self.stats['dirs_removed'] += 1
            return
        
        try:
            shutil.rmtree(dir_path, ignore_errors=True)
            self.stats['dirs_removed'] += 1
        except Exception as e:
            print(f"Error removing {dir_path}: {str(e)}")

    def _remove_file(self, file_path):
        """Remove a single file"""
        print(f"{'Would remove' if self.dry_run else 'Removing'} file: {file_path}")
        
        if self.dry_run:
            self.stats['files_removed'] += 1
            return
        
        try:
            file_path.unlink(missing_ok=True)
            self.stats['files_removed'] += 1
        except Exception as e:
            print(f"Error removing {file_path}: {str(e)}")

    def clean(self, deep=False):
        """Execute cleanup operations"""
        # Process directories that need to be emptied
        for dir_path in self._resolve_patterns(CLEAN_CONFIG["clean_inside_dirs"]):
            if dir_path.exists() and dir_path.is_dir():
                self._clean_directory_contents(dir_path)
            else:
                print(f"Directory not found: {dir_path} (skipping)")
        
        # Process directories that need to be purged
        for dir_path in self._resolve_patterns(CLEAN_CONFIG["purge_dirs"]):
            if dir_path.exists() and dir_path.is_dir():
                self._purge_directory(dir_path)
        
        # Process file cleaning
        for file_path in self._resolve_patterns(CLEAN_CONFIG["file_patterns"]):
            if file_path.exists() and file_path.is_file():
                self._remove_file(file_path)
        
        # Deep cleaning extensions
        if deep:
            for dir_path in self._resolve_patterns(CLEAN_CONFIG["deep_clean_additions"]['dirs']):
                if dir_path.exists() and dir_path.is_dir():
                    self._purge_directory(dir_path)
            
            for file_path in self._resolve_patterns(CLEAN_CONFIG["deep_clean_additions"]['files']):
                if file_path.exists() and file_path.is_file():
                    self._remove_file(file_path)
        
        # Print statistics
        print("\nCleanup Summary:")
        print(f"• Directories removed: {self.stats['dirs_removed']}")
        print(f"• Files removed: {self.stats['files_removed']}")
        print(f"• Directories cleaned: {self.stats['contents_cleaned']}")

def main():
    parser = argparse.ArgumentParser(description="Enhanced Directory Cleaner")
    parser.add_argument("--dry-run", action="store_true", help="Simulate cleanup operations")
    parser.add_argument("--deep", action="store_true", help="Perform deep cleanup")
    args = parser.parse_args()
    
    cleaner = SmartCleaner(dry_run=args.dry_run)
    cleaner.clean(deep=args.deep)
    
    print("\n" + "="*50)
    print("CLEANUP COMPLETED".center(50))
    print("="*50)

if __name__ == "__main__":
    main()