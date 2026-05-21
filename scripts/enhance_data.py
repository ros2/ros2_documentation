"""
Data structures and pure functions for tracking enhancement results and computing metrics.

This module provides a functional-programming-oriented core for managing analysis results
and deriving metrics. It is independent of the domain logic (e.g. RST file handling, OpenAI integration)
and can be reused in other contexts.
"""

from typing import NamedTuple, Dict, Set, List, Optional


class EnhanceMetrics(NamedTuple):
    """
    Immutable data structure representing analysis metrics derived from enhancement results.
    
    Attributes:
        counts_by_analysis: Dictionary mapping analysis types to their value counts.
            Example: {"content-type": {"task": 5, "concept": 3, "reference": 2}}
        files_with_results_count: Number of files that had analysis results.
        updated_files_count: Number of files that had metadata successfully updated.
    """
    counts_by_analysis: Dict[str, Dict[str, int]]
    files_with_results_count: int
    updated_files_count: int


class EnhanceData(NamedTuple):
    """
    Immutable data structure representing enhancement results.
    
    Attributes:
        results: Dictionary mapping file paths to analysis results.
            Format: {file_path: {analysis_type: result_value}}
        updated_files: Set of file paths that had metadata successfully updated.
    """
    results: Dict[str, Dict[str, str]]
    updated_files: Set[str]


def get_total_analysis_count(metrics: EnhanceMetrics) -> int:
    """
    Calculate the total number of analysis results across all analysis types.
    
    Note: Files with multiple analysis types contribute multiple counts.
    For unique file count, use metrics.files_with_results_count instead.
    
    Args:
        metrics: The metrics structure to analyse.
        
    Returns:
        Total count of all analysis results across all analysis types.
    """
    return sum(sum(counts.values()) for counts in metrics.counts_by_analysis.values())


def create_enhance_data() -> EnhanceData:
    """
    Initialise an empty EnhanceData structure.
    
    Returns:
        Empty EnhanceData with no results or updated files.
    """
    return EnhanceData(results={}, updated_files=set())


def add_analysis_result(data: EnhanceData, filename: str, analysis_type: str, result: str) -> EnhanceData:
    """
    Add an analysis result to the enhancement data.
    
    Returns a new EnhanceData instance with the added result.
    
    Args:
        data: Current enhancement data.
        filename: Path to the file (relative to repository root).
        analysis_type: Type of analysis (e.g., "content-type").
        result: Analysis result value.
        
    Returns:
        New EnhanceData with the result added.
    """

    # Creates a new EnhanceData object with the analysis result added for the given file and analysis type,
    # making copies so that original data is not changed (keeping EnhanceData immutable).
    new_results = {**data.results}
    file_results = {**new_results.get(filename, {})}
    file_results[analysis_type] = result
    new_results[filename] = file_results
    return EnhanceData(results=new_results, updated_files=data.updated_files)  # ``updated_files`` unchanged here


def mark_file_updated(data: EnhanceData, filename: str) -> EnhanceData:
    """
    Mark a file as having been successfully updated with metadata.
    
    Returns a new EnhanceData instance with the file added to updated_files.
    
    Args:
        data: Current enhancement data.
        filename: Path to the file that was updated (relative to repository root).
        
    Returns:
        New EnhanceData with the file marked as updated.
    """
    return EnhanceData(results=data.results, updated_files=data.updated_files | {filename})  # Set union adds one file path


def calculate_metrics(data: EnhanceData) -> EnhanceMetrics:
    """
    Derive metrics from enhancement data.
    
    Pure function that transforms EnhanceData into EnhanceMetrics for analysis and reporting.
    
    Args:
        data: Current enhancement data.
        
    Returns:
        EnhanceMetrics containing counts, file counts, and update counts.
    """
    counts_by_analysis: Dict[str, Dict[str, int]] = {}

    for file_results in data.results.values():
        if file_results:
            for analysis_type, result_value in file_results.items():
                clean_value = result_value.strip().lower()  # Normalise so ``Task`` and ``task`` aggregate together
                if analysis_type not in counts_by_analysis:
                    counts_by_analysis[analysis_type] = {}
                counts_by_analysis[analysis_type][clean_value] = counts_by_analysis[analysis_type].get(clean_value, 0) + 1

    files_with_results_count = sum(1 for file_results in data.results.values() if file_results)  # Files with at least one non-empty result dict

    return EnhanceMetrics(
        counts_by_analysis=counts_by_analysis,
        files_with_results_count=files_with_results_count,
        updated_files_count=len(data.updated_files)  # Distinct files whose RST was rewritten on disk
    )


def get_files_with_results(data: EnhanceData) -> List[str]:
    """
    Get list of file paths that had analysis results.
    
    Args:
        data: Current enhancement data.
        
    Returns:
        List of file paths with at least one analysis result.
    """
    return [filename for filename, file_results in data.results.items() if file_results]


def get_updated_files(data: EnhanceData) -> List[str]:
    """
    Get list of file paths that had metadata successfully updated.
    
    Args:
        data: Current enhancement data.
        
    Returns:
        List of file paths that were updated with metadata.
    """
    return list(data.updated_files)


def is_file_updated(data: EnhanceData, filename: str) -> bool:
    """
    Check if a file was successfully updated with metadata.
    
    Args:
        data: Current enhancement data.
        filename: Path to the file to check (relative to repository root).
        
    Returns:
        True if the file was updated, False otherwise.
    """
    return filename in data.updated_files


def get_analysis_types(data: EnhanceData) -> List[str]:
    """
    Get list of all analysis types performed.
    
    Args:
        data: Current enhancement data.
        
    Returns:
        List of unique analysis types found in results.
    """
    analysis_types: Set[str] = set()
    for file_results in data.results.values():
        analysis_types.update(file_results.keys())
    return list(analysis_types)


def get_result_for_file(data: EnhanceData, filename: str, analysis_type: str) -> Optional[str]:
    """
    Get analysis result for a specific file and analysis type.
    
    Args:
        data: Current enhancement data.
        filename: Path to the file (relative to repository root).
        analysis_type: Type of analysis (e.g., "content-type").
        
    Returns:
        Analysis result or None if not found.
    """
    return data.results.get(filename, {}).get(analysis_type)


def get_results_for_file(data: EnhanceData, filename: str) -> Dict[str, str]:
    """
    Get all analysis results for a specific file.
    
    Args:
        data: Current enhancement data.
        filename: Path to the file (relative to repository root).
        
    Returns:
        Dictionary of analysis results for the file, or empty dict if not found.
    """
    return data.results.get(filename, {})  # Consumed by ``update_meta_rst_files`` as ``.. meta::`` field names
